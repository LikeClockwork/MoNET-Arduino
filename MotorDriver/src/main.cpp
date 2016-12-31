#include <Arduino.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <AltSoftSerial.h>

//Set up our pwm expander boards and soft serial
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
AltSoftSerial serial2; // RX, TX

//Serial send helper function
void send( String data ) {
  serial2.println(data);
}


//Hold our size and position
class Canvas
{
  public:
  unsigned int position[2] = {0,0}; //{ x-position, y-position}
  uint32_t max[2] = {4294967295,4294967295}; //{ x-max, y-max}
  int statusCode = 0;
  Canvas(void) {};

  void updateCode (int _statusCode) {
    statusCode = _statusCode;
  }

};

Canvas canvas;

String genStatus (void) {
  String status = "{c:" + (String)canvas.statusCode + ",x:" + String(canvas.position[0]) + ",y:" + String(canvas.position[1]) + "}";
  return status;
}


//Button handling
class Button
{
  uint8_t btnPin; //What pin are we connected to?
  boolean isMomentary; //Is this button momentary or always on?
  uint8_t btnType; // 0 = pause, 1 = e-stop, 3 = soft
  unsigned long debounce; //Debounce delay .. how long do you have to hold the button down for

  boolean buttonPressed; // 3 states: 0 = not pressed, 1 = pressed and not checked, 2 = pressed and checked
  unsigned long checkMillis; //Stores last time button was checked

  public:
  boolean status; //Is the button's "status" active or not . . . are we paused/stopped/whatever
  boolean statusSent = 0; // was button status updated on server
  Button(uint8_t _btnPin, uint8_t _btnType, boolean _isMomentary = true, unsigned long _debounce = 50) {
    btnPin = _btnPin;
    isMomentary = _isMomentary;
    btnType = _btnType;
    debounce = _debounce;

    pinMode(btnPin,  INPUT_PULLUP);
    checkMillis = 0;
  }

  boolean state (void) {
      return digitalRead(btnPin);
  }

  void changeStatus(boolean targetStatus = 1) {
    if(isMomentary == 1 && targetStatus == 1) {
      //if momentary, toggle status
      status = status == 0 ? 1 : 0;
    } else if (isMomentary == 0) {
      //if latching, status is always active if button is down.
      status = targetStatus;
    }

    if(targetStatus == 1) {
      //Serial.print("Button on pin ");
      //Serial.print(btnPin);
      //Serial.print(" a status of ");
      //Serial.println(status);
    }
    if(status == 0 ) {
      statusSent = 0;
    }
    return;
  }

  boolean check() {
      unsigned long currentMillis = millis();

      //Is button pressed (gone to ground)
      if( state() == 0 ) {
        //Have we read it as pressed yet?
        if(buttonPressed == 0 ) {
          //If not mark it as pressed
          checkMillis = millis();
          buttonPressed = 1;
        } else if ( buttonPressed == 1 && (unsigned long)(currentMillis - checkMillis) > debounce ) {
          //Momentary buttons ignore the target status and just toggle. Only hard buttons take a hard value.
          changeStatus(1);
          buttonPressed = 2;
        }
      } else {
        //button is up, see above about momentary buttons
        changeStatus(0);
        buttonPressed = 0;
      }
      return status;
  }

};

Button pause(A3, 0);
Button eStop(A2,1,false);
Button soft(12,2);

//our stepper motor class
class Stepper
{
public:
  uint8_t drivePin; //NOTE: This will be on the MPC!
  uint8_t directionPin;
  boolean xOrY; //0 = x, 1 = y;
  unsigned long timer = 0;
  boolean reverse = false;
  public:
  unsigned long maxSpeed; //Max motor drive speed

  public:
  Stepper( uint8_t _drivePin, uint8_t _directionPin, boolean _xOrY, unsigned long _maxSpeed = 500, boolean _reverse = false) {
    drivePin = _drivePin;
    directionPin = _directionPin;
    xOrY = _xOrY;
    maxSpeed = _maxSpeed;
    reverse = _reverse;

    //Setup pin
    pinMode(drivePin, OUTPUT);
    pinMode(directionPin, OUTPUT);
  }

  void drive(
    boolean direction, //0 = left/up, 1 = right/down
    unsigned long speed, //pulse speed, microseconds
    boolean skip = 0
) {
    //unsigned long currentMicros = micros();

    boolean driveDir = reverse == 1 ? !direction : direction;

    //if((unsigned long)(currentMicros - timer) > speed) {

      //unsigned long pulseDelay = speed/2 < maxSpeed/2 ? maxSpeed/2 : speed/2;

      //Set direction
      digitalWrite(directionPin, driveDir);



      //update position


      //pulse the motor
      if( !skip ) {

        int step = 0; //increment or decrement postion
        if(direction == 0) {
          step--;
        } else {
          step++;
        }

        digitalWrite(drivePin,HIGH);
        canvas.position[xOrY] += step;
      }


      //Actually do the update.
      //timer = currentMicros;
    //}

    return;

  }

  void pulse (void) {
    digitalWrite(drivePin, LOW);

  }


};

Stepper* motors[2];


class Pump
{
  uint8_t pumpPin; //Pin that the pump is on on the PWM driver
  float minValue = 100; //These pumps have a minimum speed below which the don't turn, so we need to compensate for that.
  float maxValue = 255; //Another calibration point, if needed.

  public:
  Pump( uint8_t _pumpPin ) {
    pumpPin = _pumpPin;
  }

  //These will drive until stopped by hitting the speed with 0
  void drive (  float speed ) {
    if (speed > 0) {
      speed = (((maxValue - minValue)/255.00) * speed) + minValue;
    }
    pwm.setPWM(pumpPin ,0, (speed / 255) * 4095);
    delay(10);
  }

};

Pump redPump(0);
Pump greenPump(1);
Pump bluePump(2);
Pump whitePump(3);
Pump blackPump(4);
Pump mixPump(5);
Pump dispensePump(6);
Pump cleanPump(7);

boolean lockPumps = 0; //Did we kill the pumps? Record it.

void killPumps (void) {
  redPump.drive(0);
  greenPump.drive(0);
  bluePump.drive(0);
  whitePump.drive(0);
  blackPump.drive(0);
  mixPump.drive(0);
  dispensePump.drive(0);
  cleanPump.drive(0);
  lockPumps = 1;
}


class Limit
{
  uint8_t limitPin;
  public:
  boolean limitSent = 0; //Was limit prompt sent to server?
  boolean limitFound = 0;
  boolean limitSet = 0;
  boolean limitType = 0; //0 = min, 1 = max
  boolean direction = 0; //0 = x, 1 = y
  unsigned int limitValue = 0; //Saved limit amount
  int backoff = 0; //Steps to back off of limit
  int countBackoff = 0; //storage for backout
  Button limitSwitch;
  Limit(boolean _direction, boolean _limitType, uint8_t _limitPin, int _backoff = 100, unsigned long _debounce = 25)
    //Attach one of the limit switches as a non-momentary button.
    : limitSwitch(_limitPin, 0, false, _debounce)
  {
    limitPin = _limitPin;
    direction = _direction;
    limitType = _limitType;
    //Switch backoff direction if it's a max.
    backoff = _backoff;

  }

  void reset(void) {
    limitFound = 0;
    limitSet = 0;
  }

  void findLimit() {
    //check the switch
    //If limit hasn't been found yet
    if(limitSwitch.check() == 0 && limitFound == 0) {
      //Drive motor until the switch is activated.
      motors[direction] -> drive(limitType, 1);
      delayMicroseconds(200);
      motors[direction] -> pulse();
      delayMicroseconds(50);
    //Switch is activated, set limitFound
    } else if( limitSwitch.check() == 1 && limitFound == 0) {
      limitFound = 1;
      countBackoff = backoff;
    //Now backoff predetermined amount, and then set the limit.
    } else if ( limitFound == 1 && limitSet == 0) {
      if( limitType == 0 ) {
        motors[direction] -> drive(1,1);
        delayMicroseconds(200);
        motors[direction] -> pulse();
        delayMicroseconds(50);
      } else {
        motors[direction] -> drive(0,1);
        delayMicroseconds(200);
        motors[direction] -> pulse();
        delayMicroseconds(50);
      }
      countBackoff--;

      //Undo backoff, now just look for limit switch not to be depressed.
      if(limitSwitch.check() == 0) {
        limitSet = 1;
        limitSent = 0;
        soft.status = 0;
        if (limitType == 1) {
          canvas.max[direction] = canvas.position[direction];
        } else {
            canvas.position[direction] = 0;
        }
        Serial.print("Limit found at ");
        Serial.println(canvas.position[direction]);
        send("{l:" + (String)canvas.position[direction] +  "}");
        soft.status = 0;
        pause.status = 0;
      }
    }
  }


};

Limit xMin(0, 0, A0, 500);
Limit xMax(0, 1, 3, 500);
Limit yMin(1, 0, 2, 500);
Limit yMax(1, 1, A1, 500);









boolean allLimitsPassed = false;

//This function performs a check of all our serious stop conditions. These will prevent any additional code from running.
//REMEMBER TO ADD A KILL FUNCTION IN HERE!
boolean passedCriticalStops (void) {

    //eStop has been pushed! Stop everything!
    if( eStop.check() == 1 ) {
      killPumps();
      pause.status = 1;
      if(eStop.statusSent == 0 ) {
        Serial.println("eStop pressed.");
        canvas.updateCode(4);
        eStop.statusSent = 1;
      }
      return 0;



      //Limit switches only come online after calibration
  } else if (allLimitsPassed == true) {
    //xMin is set, however somehow the xMin limit switch has been tripped! Stop everything!
      if (xMin.limitSwitch.check() == 1 && xMin.limitSet == 1){
        pause.status = 1;
        killPumps();
        if(xMin.limitSwitch.statusSent == 0 ) {
          canvas.updateCode(13);
          xMin.limitSwitch.statusSent = 1;
        }
        return 0;

      //xMax is set, however somehow the xMax limit switch has been tripped! Stop everything!
      } else if (xMax.limitSwitch.check() == 1 && xMax.limitSet == 1){
        pause.status = 1;
        killPumps();
        if(xMax.limitSwitch.statusSent == 0 ) {
          canvas.updateCode(14);
          xMax.limitSwitch.statusSent = 1;
        }
        return 0;

      //yMin is set, however somehow the yMin limit switch has been tripped! Stop everything!
      } else if (yMin.limitSwitch.check() == 1 && yMin.limitSet == 1){
        pause.status = 1;
        killPumps();
        if(yMin.limitSwitch.statusSent == 0 ) {
          canvas.updateCode(15);
          yMin.limitSwitch.statusSent = 1;
        }
        return 0;

      //yMax is set, however somehow the yMax limit switch has been tripped! Stop everything!
      } else if (yMax.limitSwitch.check() == 1 && yMax.limitSet == 1){
        pause.status = 1;
        killPumps();
        if(yMax.limitSwitch.statusSent == 0 ) {
          canvas.updateCode(16);
          yMax.limitSwitch.statusSent = 1;
        }
        return 0;
      //Machine is in a pause condition! Stop everything!
      } else if ( pause.check() == 1) {
        killPumps();
        if(pause.statusSent == 0 ) {
          Serial.println("Paused.");
          canvas.updateCode(3);
          pause.statusSent = 1;
        }
        return 0;
      }

    //All checks passed!
    }
      return 1;

}

int gcd(unsigned int x, unsigned int y) {
    /*;
        a = qb + r,  0 <= r < b

        a => dividend, q => quotient, b => divisor, r => remainder
    */
    if (x == y) {
        return 1 /*or y*/;
    }

    unsigned int dividend = x, divisor = y;
    unsigned int quotient = 0;
    unsigned int remainder = 0;

    do {
        remainder = dividend % divisor;
        quotient = dividend / divisor;

        if(remainder) {
            dividend = divisor;
            divisor = remainder;
        }
    }
    while(remainder);

    return divisor;
}

int lcm(unsigned int x, unsigned int y) {
    /*
        lcm(x,y) = (x * y) / gcd(x,y)
    */
    return x == y ? 1 /*or y*/ : (x * y) / gcd(x,y);
}



//Connect to web, fetch and follow instructions!
class Command
{
  unsigned long checkMillis = 0;//Delay checker
  long deltaX; //difference between old and new x position
  long deltaY; //difference between old and new y position
  unsigned long speedX; //X vector for speed (lower is faster)
  unsigned long speedY; //Y vector for speed
  int xTick;
  int yTick;
  boolean xDirection;
  boolean yDirection;
  unsigned long checkMicrosX = 0;
  unsigned long checkMicrosY = 0;

  public:
    boolean active = 0; //Is the command active
    boolean firstTick = 0; //Is this the first call of the doCommand function.
    unsigned long commandId = 0; //command ID
    uint16_t
      x = 0,      // x destination
      y = 0;    // y destination
    uint16_t
      red = 0,    //amount to run paint motors
      green = 0,
      blue = 0,
      white = 0,
      black = 0,
      mix = 0,
      dispense = 0,
      clean = 0,
      speed = 0;  //speed to move at to destination
      boolean madeReq = 0;
      char cmdString[300];
      int charRec = 0;
  public:
  Command(void) {}


  boolean requestCommand (void) {
    unsigned long currentMillis = millis();

    //Don't request anything if a command is already active
    if( active == 1 ) {
      return 1;
    }

    //If a request was made, but we aren't active, request again.
    if((unsigned long)(currentMillis - checkMillis) > 5000 && madeReq == 1) {
      madeReq = 0;
      checkMillis = currentMillis;
    }

    //make the request and reset the timer
    if (madeReq == 0 ) {
      canvas.updateCode(17);
      //Serial.println("Requesting command.");
      //send("{m:\"NEED_CMD\"}");
      madeReq = 1;
      checkMillis = currentMillis;
      return 1;
    }

}

  boolean parseCommand(String buffer) {
        //Make command active

        //load up the command parameters
        int i = buffer.indexOf(":") + 1;

        if ( buffer.substring(i, buffer.indexOf(",",i) ).toInt() == 0 ) {
          Serial.println("No command waiting");
          delay(2000);
          return 0;
        }

        //Load up the commands
        i = buffer.indexOf(",", i) + 1;
        commandId = buffer.substring(i, buffer.indexOf(",",i) ).toInt();

        i = buffer.indexOf(",", i) + 1;
        x = buffer.substring(i, buffer.indexOf(",",i) ).toInt();

        i = buffer.indexOf(",", i) + 1;
        y = buffer.substring(i, buffer.indexOf(",",i) ).toInt();

        i = buffer.indexOf(",", i) + 1;
        speed = buffer.substring(i, buffer.indexOf(",",i) ).toInt();

        i = buffer.indexOf(",", i) + 1;
        red = buffer.substring(i, buffer.indexOf(",",i) ).toInt();

        i = buffer.indexOf(",", i) + 1;
        green = buffer.substring(i, buffer.indexOf(",",i) ).toInt();

        i = buffer.indexOf(",", i) + 1;
        blue = buffer.substring(i, buffer.indexOf(",",i) ).toInt();

        i = buffer.indexOf(",", i) + 1;
        black = buffer.substring(i, buffer.indexOf(",",i) ).toInt();

        i = buffer.indexOf(",", i) + 1;
        white = buffer.substring(i, buffer.indexOf(",",i) ).toInt();

        i = buffer.indexOf(",", i) + 1;
        mix = buffer.substring(i, buffer.indexOf(",",i) ).toInt();

        i = buffer.indexOf(",", i) + 1;
        dispense = buffer.substring(i, buffer.indexOf(",",i) ).toInt();

        i = buffer.indexOf(",", i) + 1;
        clean = buffer.substring(i, buffer.indexOf(",",i) ).toInt();


        //Get distance to move
        deltaX = (long)x - (long)canvas.position[0];
        deltaY = (long)y - (long)canvas.position[1];


        //Get magnitude of moves.
        unsigned long deltaXTemp = deltaX < 0 ? deltaX * -1 : deltaX;
        unsigned long deltaYTemp = deltaY < 0 ? deltaY * -1 : deltaY;

        //Get x and y speed vectors, minimum interval is in motor class
        float rads = atan2(deltaYTemp, deltaXTemp);
        float crads = cosf(rads);
        float srads = sinf(rads);
        speedX = (long)( crads * (float)speed);
        speedY = (long)( srads * (float)speed);

        //Refigure speed if it's maxing out
        long xMax = motors[0] -> maxSpeed;
        long yMax = motors[1] -> maxSpeed;

        speedX = speedX < 0 ? speedX * -1 : speedX;
        speedY = speedX < 0 ? speedY * -1 : speedY;


        Serial.println(x);
        Serial.println(y);
        Serial.print("speed: ");
        Serial.println(speed);
        Serial.print("rads: ");
        Serial.println(rads);
        Serial.print("cos: ");
        Serial.println(crads);
        Serial.print("sin: ");
        Serial.println(srads);
        Serial.print("speedX prerefactor: ");
        Serial.println(speedX);
        Serial.print("speedY prerefactor: ");
        Serial.println(speedY);
        Serial.print("deltaX pos: ");
        Serial.println(deltaXTemp);
        Serial.print("deltaY pos: ");
        Serial.println(deltaYTemp);


        if( speedX < xMax || speedY < yMax) {
          long tempSpeed = xMax >= yMax ? xMax : yMax;
          //long tempSpeed = speedX < speedY ? xMax/srads : yMax / crads;
          speedX = (unsigned long)( crads * tempSpeed);
          speedY = (unsigned long)( srads * tempSpeed);
        }

        speedX = speedX < 0 ? xMax : speedX;
        speedY = speedY < 0 ? yMax : speedY;
        speedX = speedX > 100000 ? speed : speedX;
        speedY = speedY > 100000 ? speed : speedY;


        unsigned int absDx = abs(deltaX);
        unsigned int absDy = abs(deltaY);
        /*
        while ( absDx > 20 && absDy > 20) {
          absDx /= 3;
          absDy /= 3;
        }

        xTick = absDx;
        yTick = absDy;

        xTick = xTick == 0 ? 1 : xTick;
        yTick = yTick == 0 ? 1 : yTick;
        */

        xDirection = deltaX > 0 ? 1 : 0;
        yDirection = deltaY > 0 ? 1 : 0;

        Serial.println("Got command, closing connection");
        Serial.print("deltaX: ");
        Serial.println(deltaX);
        Serial.print("deltaY: ");
        Serial.println(deltaY);
        Serial.print("speedX: ");
        Serial.println(speedX);
        Serial.print("speedY: ");
        Serial.println(speedY);
        Serial.print("xTick: ");
        Serial.println(xTick);
        Serial.print("yTick: ");
        Serial.println(yTick);
        canvas.updateCode(2);
        active = 1;
        return 1;

  }

  void doCommand (void) {

      //Unlock the pumps if for some reason they were locked. This is so if we are continuing after pause, we're OK.
      if( lockPumps == 1 ) {
        firstTick = 0;
        lockPumps = 0;
      }

      //Was request to pump paint on this move made? Only do this on first pass through this function so you don't spam i2c.
      if( red + green + blue + black + white + blue + mix + dispense + clean > 0 && firstTick == 0) {
          redPump.drive(red);
          greenPump.drive(green);
          bluePump.drive(blue);
          whitePump.drive(white);
          blackPump.drive(black);
          mixPump.drive(mix);
          dispensePump.drive(dispense);
          cleanPump.drive(clean);
          firstTick  = 1;
      }

      unsigned long currentMicros = micros();

      if( (unsigned long)(currentMicros - checkMicrosX) > speedY && x != canvas.position[0]) {
        motors[0] -> drive ( xDirection, speed/2 );
        checkMicrosX = micros();
      }


      if( (unsigned long)(currentMicros - checkMicrosY) > speedX && y != canvas.position[1]) {
        motors[1] -> drive ( yDirection, speed/2 );
        checkMicrosY = micros();
      }

        delayMicroseconds(10);
        motors[0] -> pulse();
        motors[1] -> pulse();
        delayMicroseconds(10);


      /*
      unsigned long dx = abs(deltaX);
      unsigned long dy = abs(deltaY);

      unsigned long i;
      unsigned long over;


      int loopLength = xTick >= yTick ? xTick : yTick;
      int skipSteps = xTick >= yTick ? xTick - yTick : yTick - xTick;
      int interval = loopLength / skipSteps;

      int j = interval;

      for( int i = 1; i < loopLength + 1; i++) {
        if( xTick >= yTick) {
          if(x != canvas.position[0] ) {
            motors[0] -> drive ( xDirection, speed/2 );
          }
          j--;
          if( j == 0  && y != canvas.position[1]) {
            motors[1] -> drive ( yDirection, speed/2 );
            j = interval;
          }
        } else {
          if(y != canvas.position[1] ) {
            motors[1] -> drive ( xDirection, speed/2 );
          }
          j--;
          if( j == 0  && x != canvas.position[0]) {
            motors[0] -> drive ( xDirection, speed/2 );
            j = interval;
          }
        }

        delayMicroseconds(50);
        motors[0] -> pulse();
        motors[1] -> pulse();
        delayMicroseconds(50);

      }
      */


      if( x == canvas.position[0] && y == canvas.position[1] ) {
        finishCommand();
        firstTick = 0;
        active = 0;
        madeReq = 0;
        return;
      }

        return;

  }

  //If the move is complete, contact the server and let it know you're done.
  //Shut down the paint pumps
  void finishCommand() {
    killPumps();
    Serial.println("Command complete");
    canvas.updateCode(18);
  }

};

Command command;

/*
Status codes
0 = none
1 = idle
2 = executing command
3 = paused
4 = eStopped
5 = prompt for xMin
6 = prompt for xMax
7 = prompt for yMin
8 = prompt for yMax
9 = finding for xMin
10 = finding for xMax
11 = finding for yMin
12 = finding for yMax
13 = exceeded xMin
14 = exceeded xMax
15 = exceeded yMin
16 = exceeded yMax
17 = request Command
*/


//When the machine first starts, it won't know where it is, so it should find it's limits automagically.
//Again, this will prevent any main code from running.
boolean foundLimits (void) {

  if( xMin.limitSet == 0) {
    if(xMin.limitSent == 0 ) {
      Serial.println("Waiting on xMin.");
      canvas.updateCode(5);
      xMin.limitSent = 1;
    }
    //Wait for user to prompt for calibration
    if(soft.check() == 1) {
      canvas.updateCode(9);
      xMin.findLimit();
    }
    return 0;
  } else if( xMax.limitSet == 0) {
    if(xMax.limitSent == 0 ) {
      Serial.println("Waiting on xMax.");
      canvas.updateCode(6);
      xMax.limitSent = 1;
    }
    //Wait for user to prompt for calibration
    if(soft.check() == 1) {
      canvas.updateCode(10);
      xMax.findLimit();
    }
    return 0;
  } else if( yMin.limitSet == 0) {
    if(yMin.limitSent == 0 ) {
      Serial.println("Waiting on yMin.");
      canvas.updateCode(7);
      yMin.limitSent = 1;
    }
    if(soft.check() == 1) {
      canvas.updateCode(11);
      yMin.findLimit();
    }
    return 0;
  } else if( yMax.limitSet == 0) {
    if(yMax.limitSent == 0 ) {
      Serial.println("Waiting on yMax.");
      canvas.updateCode(8);
      yMax.limitSent = 1;
    }
    //Wait for user to prompt for calibration
    if(soft.check() == 1) {
      canvas.updateCode(12);
      yMax.findLimit();
    }
    return 0;
  } else {
    allLimitsPassed = true;
    return 1;
  }
}

void setup (void) {

  Serial.begin(57600);
  serial2.begin(28800);
  Serial.print("Serial Begin");

  delay(500);

    Wire.begin();


    pwm.begin();
    Serial.print("PWM Start");


    pwm.setPWMFreq(1000);
    Serial.print("PWM Set Freq");




    delay(1000);

    Serial.print("Ready");

    //Initialize the stepper motors
    motors[0] = new Stepper(7, 6, 0, 1000, true);
    motors[1] = new Stepper(5, 4, 1, 1000);


}

//1000 steps = 2.218" x
//1000 steps = 2.410" y


//int counter = 500;
//boolean direction;

unsigned long reqTimer = 0;

//char buffer[200] = "";
//int msgLength = 0;

void loop (void) {

  //Only preceed if stop conditions are clear
  if (passedCriticalStops()) {
    //Only actually do stuff if our calibrations are set.
    if( foundLimits() ) {
      //If there's a command waiting ...
      if( command.active == 1) {
        //Do a cycle of it ...
        command.doCommand();
      } else {
        //Request new command from WiFiMCU
        command.requestCommand();
      }

    }

  }

  //Parse incoming commands from WiFiMCU, ignore commands that come in while doing command
  if (serial2.available() && canvas.statusCode != 2) {
          // read the incoming byte:
          String buffer = serial2.readStringUntil('\n');

          Serial.println(buffer);


          //Ghetto serial router. Do stuff based on prefix
          if( buffer.substring(0, buffer.indexOf(":")) == "CMD") {
            Serial.println("Command started ...");
            command.parseCommand(buffer);
          }

  }





  unsigned long currentMillis = millis();

  //Send a position update every second as long as we aren't waiting on a command
  if((unsigned long)(currentMillis - reqTimer) > 1000 && canvas.statusCode != 1) {
    String message = genStatus();
    serial2.println( message );
    Serial.println (message);
    if ( canvas.statusCode == 17 ) {
      canvas.updateCode(1);
    }
    reqTimer = currentMillis;
  }


  /*
  digitalWrite(2, direction);
  digitalWrite(3,HIGH);
  delayMicroseconds(1000);
  digitalWrite(3,LOW);
  delayMicroseconds(1000);

  digitalWrite(4, direction);
  digitalWrite(5,HIGH);
  delayMicroseconds(1000);
  digitalWrite(5,LOW);
  delayMicroseconds(1000);

  counter--;

  if( counter == 0 ) {
    direction =  direction == 1 ? 0 : 1;
    counter = 500;
  }
  */
}
