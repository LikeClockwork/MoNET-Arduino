#include "conf.h"
#include <Wire.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <SocketIOClient.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>
#include "Adafruit_MCP23017.h"
//#include "bitmaps.h"

//Set up our display, pwm, and port expander boards
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_MCP23017 mcp;
//Adafruit_SSD1306 display(D0);
SocketIOClient ioClient;

extern String RID;
extern String Rname;
extern String Rcontent;

//Create a buffer for JSON responses
StaticJsonBuffer<200> jsonBuffer;

//WiFi, Server and Hardware settings
const char* myssid = MYSSID;
const char* mypass = MYPASS;
const char* myhost = MYHOST;
const char* hardwareID = HARDWAREID;
const int httpPort = HTTPPORT;

//Join WiFi based on prefs in the conf.h file.
void joinWiFi() {
  //Don't rejoin if you are already connected!
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println();
    Serial.println();
    Serial.print("WiFi connecting to ");
    Serial.println( MYSSID );

      WiFi.begin(MYSSID, MYPASS);

       while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }

      Serial.println("");
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
    }
}

String contactServer ( String params ) {
  //Check to see if WiFi is connected, if not reconnect.
  joinWiFi();

  //Open a connection to the server.
  Serial.print("Connecting to ");
  Serial.println(myhost);
  WiFiClient client;
  //Bail out and try again if the connection fails.
  if (!client.connect(myhost, httpPort)) {
    Serial.println("Connection failed!");
    return "";
  }
  //Sent the request to the server. This will hit an endpoint which will reply with the next command.
  String url = String("/robotcontol/") + hardwareID + params;
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + myhost + "\r\n" +
               "Accept: */*\r\n" +
               "User-Agent: Mozilla/4.0 (compatible; esp8266 Lua; )\r\n" +
               "Connection: close\r\n\r\n");
  delay(10);

  //Wait patiently for server to respond.
  while(!client.available()){
    yield();
  }

  // Read all the lines of the reply from server and print them to Serial
  String response;
  while(client.available()){
     response = client.readStringUntil('\r');
     yield();
  }
  Serial.println(response);
  return response;
}

//Hold our size and position
class Canvas
{
  public:
  unsigned int position[2] = {0,0}; //{ x-position, y-position}
  unsigned int max[2] = {4294967295,4294967295}; //{ x-max, y-max}
  Canvas(void) {};
};

Canvas canvas;

//Button handling
class Button
{
  uint8_t btnPin; //What pin are we connected to?
  boolean isMomentary; //Is this button momentary or always on?
  uint8_t btnType; // 0 = pause, 1 = e-stop, 3 = soft
  int debounce; //Debounce delay .. how long do you have to hold the button down for

  boolean buttonPressed; // 3 states: 0 = not pressed, 1 = pressed and not checked, 2 = pressed and checked
  unsigned long checkMillis; //Stores last time button was checked

  public:
  boolean status; //Is the button's "status" active or not . . . are we paused/stopped/whatever
  Button(uint8_t _btnPin, uint8_t _btnType, boolean _isMomentary = true, int _debounce = 50) {
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
      Serial.print("Button on pin ");
      Serial.print(btnPin);
      Serial.print(" a status of ");
      Serial.println(status);
    }
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

Button pause(10, 0);
Button eStop(9,1,false);
Button soft(D7,2);

//our stepper motor class
class Stepper
{
  uint8_t drivePin; //NOTE: This will be on the MPC!
  uint8_t directionPin;
  boolean xOrY; //0 = x, 1 = y;

  public:
  Stepper( uint8_t _drivePin, uint8_t _directionPin, boolean _xOrY) {
    drivePin = _drivePin;
    directionPin = _directionPin;
    xOrY = _xOrY;

    //Setup pin
    mcp.pinMode(drivePin, OUTPUT);
    mcp.pinMode(directionPin, OUTPUT);
  }

  void drive(
    boolean direction, //0 = left/up, 1 = right/down
    int speed //pulse speed, microseconds
  ) {
    //Set direction
    mcp.digitalWrite(directionPin, direction);
    //pulse the motor
    mcp.digitalWrite(drivePin,HIGH);
    delayMicroseconds(speed);
    mcp.digitalWrite(drivePin, LOW);
    delayMicroseconds(speed);

    //update position
    int step = 0; //increment or decrement postion
    if(direction == 0) {
      step--;
    } else {
      step++;
    }
    //Actually do the update.
    canvas.position[xOrY] += step;

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
    Serial.println(speed);
    Serial.println((speed / 255) * 4095);
    pwm.setPWM(pumpPin ,0, (speed / 255) * 4095);
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

void killPumps (void) {
  redPump.drive(0);
  greenPump.drive(0);
  bluePump.drive(0);
  whitePump.drive(0);
  blackPump.drive(0);
  mixPump.drive(0);
  dispensePump.drive(0);
  cleanPump.drive(0);
}


class Limit
{
  uint8_t limitPin;
  public:
  boolean limitFound = 0;
  boolean limitSet = 0;
  boolean limitType = 0; //0 = min, 1 = max
  boolean direction = 0; //0 = x, 1 = y
  unsigned int limitValue = 0; //Saved limit amount
  int backoff = 0; //Steps to back off of limit
  int countBackoff = 0; //storage for backout
  Button limitSwitch;
  Limit(boolean _direction, boolean _limitType, uint8_t _limitPin, int _backoff = 100)
    //Attach one of the limit switches as a non-momentary button.
    : limitSwitch(_limitPin, 0, false)
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
      motors[direction] -> drive(limitType, 100);
    //Switch is activated, set limitFound
    } else if( limitSwitch.check() == 1 && limitFound == 0) {
      limitFound = 1;
      countBackoff = backoff;
    //Now backoff predetermined amount, and then set the limit.
    } else if ( limitFound == 1 && limitSet == 0) {
      if( !limitType ) {
        motors[direction] -> drive(1,100);
      } else {
        motors[direction] -> drive(0,100);
      }
      countBackoff--;
      if(countBackoff < 1) {
        limitSet = 1;
        soft.status = 0;
        if (limitType == 1) {
          canvas.max[direction] = canvas.position[direction];
        } else {
            canvas.position[direction] = 0;
        }
        Serial.print("Limit found at ");
        Serial.println(canvas.position[direction]);
        soft.status = 0;
        pause.status = 0;
      }
    }
  }


};

Limit xMin(0, 0, D2);
Limit xMax(0, 1, D1);
Limit yMin(1, 0, D5);
Limit yMax(1, 1, D6);

//Connect to web, fetch and follow instructions!
class Command
{
  unsigned long checkMillis = 0;//Delay checker
  int deltaX; //difference between old and new x position
  int deltaY; //difference between old and new y position
  int speedX; //X vector for speed (lower is faster)
  int speedY; //Y vector for speed
  boolean active = 0; //Is the command active
  public:
    boolean firstTick = 0; //Is this the first call of the doCommand function.
    unsigned long commandId; //command ID
    unsigned int
      x = 0,      // x destination
      y = 0;    // y destination
    uint8_t
      red = 0,    //amount to run paint motors
      green = 0,
      blue = 0,
      white = 0,
      black = 0,
      mix = 0,
      dispense = 0,
      clean = 0,
      speed = 0;  //speed to move at to destination
    unsigned int errorCount;
      String JSON;
      JsonObject& root = jsonBuffer.createObject();

  public:
  Command(void) {}

  //This is our Web Sockets display
  void wsDisplay (void) {
    //Join WiFi if not connected
    while( !WL_CONNECTED ) {
      joinWiFi();
      yield();
    }

    unsigned long wscheckMillis = 0;

    //Wait for proof of connection. if it doesn't come, reconnect and try again on next tick.
    if(ioClient.monitor()){
    /*
    while(!ioClient.monitor()) {
      unsigned long wscurrentMillis = millis();
      yield();
      if( (unsigned long)(wscurrentMillis - wscheckMillis) > 10000 ) {
        ioClient.disconnect();
        errorCount++;
        wscheckMillis = wscurrentMillis;
        if (!ioClient.connect(const_cast<char*>(myhost), httpPort) ) {
          Serial.println("connection failed");
        }
        return;
      }
    }
    */
      //Actually send data.
      // clear the json buffer
      JSON = "";

      //Load up the JSON
      root["xPos"] = 0;
      root["yPos"] = 0;
      root.printTo(JSON);

      Serial.println(JSON);
      Serial.print("Reconnect Count: ");
      Serial.println(errorCount);
      ioClient.sendJSON("MCU", JSON);

    } else {
      errorCount++;
      if (errorCount > 5 ) {
        Serial.println("Attempting reconnect");
        if (!ioClient.connect(const_cast<char*>(myhost), httpPort) ) {
          Serial.println("connection failed");
        } else {
          errorCount = 0;
        }
      }
      wsDisplay();
    }

  }


  //If the move is complete, contact the server and let it know you're done.
  //Shut down the paint pumps
  boolean finishCommand() {
    killPumps();

    /*
      Returns something like this:
      response = {
        "commandId" : 647827486,
        "status" : "complete";

    */

    //Actually make the resquest and try again if it fails;
    String request = "/" + (String)commandId + "/complete";
    String response = contactServer(request);
    if( response == "") {
      delay(5000);
      finishCommand();
    }
    //Otherwise, parse the response
    JsonObject& res = jsonBuffer.parseObject(response);

    String status = res["status"];
    Serial.println(status);

    if( status != "complete") {
      delay(5000);
      finishCommand();
    } else {
      return 1;
    }
  }



  //Move and pump paint. May need to add a hold at start and finish for the paint to finish
  boolean doCommand (void) {
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

      //Paint set, we're moving out!
      active = 1;

      if( x != canvas.position[0] ) {
        boolean directionX = deltaX > 0 ? 1 : 0;
        motors[0] -> drive( directionX, speedX );
      }

      if( y != canvas.position[1] ) {
        boolean directionY = deltaY > 0 ? 1 : 0;
        motors[1] -> drive( directionY, speedY );
      }

      if( x == canvas.position[0] && y == canvas.position[1] ) {
        finishCommand();
        firstTick = 0;
        active = 0;
        return 1;
      } else {
        return 0;
      }

  }


  //Make a request to the Project MoNET server and fetch some instructions
  boolean fetchCommand (void) {
    unsigned long currentMillis = millis();

    //if the current command is active, we don't need another one
    if( active == 1 ) {
      return 1;

    //Non blocking delay of 5 seconds for command requests
    } else if((unsigned long)(currentMillis - checkMillis) > 5000) {

      //Contact the server
      //Server should respond with JSON data that looks like this
      /*
        command = {
          "commandReady": 1,
          "commandId" : 647827486,
          "x" : 123456, //x destination
          "y" : 123456, //y destination
          "speed" : 234, //speed at which the steppers will move for this command
          "red" : 123, //paint pump rates
          "green" : 123,
          "blue" : 123,
          "white" : 123,
          "black" : 123,
          "mix" : 123,
          "dispense" : 123,
          "clean" : 123,
          "status" : "queued"
      }

      or


      command = {
          "commandReady" : 0
      }

      If there's nothing to do.
      */

      //Actually make the resquest, bail out and try again if it fails;
      String response = contactServer("");
      if( response == "") {
        active = 0;
        return 0;
      }
      //Otherwise, parse the response
      JsonObject& res = jsonBuffer.parseObject(response);

      //Check if command is waiting
      if( !(boolean)res["commandReady"] ) {
        Serial.print("Connection good but no command waiting, try again later.");
        active = 0;
        return 0;
      } else {
        //load up the command object;
        commandId = (unsigned long)res["commandId"];
        x = (unsigned int)res["x"];
        y = (unsigned int)res["y"];
        speed = (unsigned int)res["speed"];
        red = (uint8_t)res["red"];
        green = (uint8_t)res["green"];
        blue = (uint8_t)res["blue"];
        black = (uint8_t)res["black"];
        white = (uint8_t)res["white"];
        mix = (uint8_t)res["mix"];
        dispense = (uint8_t)res["dispense"];
        clean = (uint8_t)res["clean"];

        //Get distance to move
        deltaX = (long)x - (long)canvas.position[0];
        deltaY = (long)y - (long)canvas.position[1];


        //Get x and y speed vectors, minimum interval is 100us
        float rads = atan2(deltaX, deltaY);
        speedX = (int)( sinf(rads) * (float)speed);
        speedY = (int)( cosf(rads) * (float)speed);

        speedX = speedX < 100 ? 100 : speedX;
        speedY = speedY < 100 ? 100 : speedX;
        Serial.println("Got command, closing connection");

        //Set the command active
        delay(10);
        return 1;
      }
    } else {
      return 0;
    }
  }
};

Command command;

/*
//OLED display stuff
class OLEDDisplay
{
  uint8_t frame = 0; //Holds the frame number;
  unsigned long checkMillis = 0; //Timekeeping
  unsigned long netMillis = 0; //Timekeeping


  public:
  OLEDDisplay(void) {}

  void reset() {
    display.clearDisplay();
    frame = 0;
  }


  //Show a full screen image instead of the usual status display
  void doFullScreen (int frameSet,  int frameRate) {
    unsigned long currentMillis = millis();

    //draw frame.
    if((unsigned long)(currentMillis - checkMillis) > frameRate) {
      //display.clearDisplay();
      uint8_t tempFrameSet = frameSet;
      //Serial.print("frame set: ");
      //Serial.println(frameSet);
      //Serial.print("frame: ");
      //Serial.println(frame);
      //Serial.print("length: ");
      //Serial.println( ((frameLookup[frameSet][1] + 1) - frameLookup[frameSet][0]));
      display.drawBitmap(1, 1,  bitmaps[frameLookup[frameSet][0] + frame], 128, 64, 1);
      //display.display();
      frame++;
      checkMillis = currentMillis;
      if(frame >= ((frameLookup[frameSet][1] + 1) - frameLookup[frameSet][0])) {
        reset();
      }

    }
  }

  void statusDisplay (int frameRate) {
    unsigned long currentMillis = millis();

    if((unsigned long)(currentMillis - netMillis) > frameRate) {
      ioClient.send("test", "message", "Test");

      netMillis = currentMillis;
    }
  }
};

OLEDDisplay oled;
*/

//kill pumps and perform a recalibration
void hardReset (void) {
  command.firstTick = 0;
  killPumps();
  xMin.reset();
  xMax.reset();
  yMax.reset();
  yMin.reset();
}


//This function performs a check of all our serious stop conditions. These will prevent any additional code from running.
//REMEMBER TO ADD A KILL FUNCTION IN HERE!
boolean passedCriticalStops (void) {

    //eStop has been pushed! Stop everything!
    if( eStop.check() == 1 ) {
      pause.status = 1;
      //oled.doFullScreen(0, 1500);
      killPumps();
      return 0;

    //Machine is in a pause condition! Stop everything!
    } else if ( pause.check() == 1) {
      //oled.doFullScreen(1, 1500);
      killPumps();
      return 0;

    //xMin is set, however somehow the xMin limit switch has been tripped! Stop everything!
    } else if (xMin.limitSwitch.check() == 1 && xMin.limitSet == 1){
      pause.status = 1;
      //oled.doFullScreen(0, 1500); //change this to limit graphic
      killPumps();
      return 0;

    //xMax is set, however somehow the xMax limit switch has been tripped! Stop everything!
    } else if (xMax.limitSwitch.check() == 1 && xMax.limitSet == 1){
      pause.status = 1;
      //oled.doFullScreen(0, 1500); //change this to limit graphic
      killPumps();
      return 0;

    //yMin is set, however somehow the yMin limit switch has been tripped! Stop everything!
    } else if (yMin.limitSwitch.check() == 1 && yMin.limitSet == 1){
      pause.status = 1;
      //oled.doFullScreen(0, 1500); //change this to limit graphic
      killPumps();
      return 0;

    //yMax is set, however somehow the yMax limit switch has been tripped! Stop everything!
    } else if (yMax.limitSwitch.check() == 1 && yMax.limitSet == 1){
      pause.status = 1;
      //oled.doFullScreen(0, 1500); //change this to limit graphic
      return 0;

    //All checks passed!
    } else {
      return 1;
    }
}




//When the machine first starts, it won't know where it is, so it should find it's limits automagically.
//Again, this will prevent any main code from running.
boolean foundLimits (void) {

  if( xMin.limitSet == 0) {
    //oled.doFullScreen(2, 1500); //change this to prompt for find limit graphic
    //Wait for user to prompt for calibration
    if(soft.check() == 1) {
      xMin.findLimit();
    }
    return 0;
  } else if( xMax.limitSet == 0) {
    //oled.doFullScreen(3, 1500); //change this to prompt for find limit graphic
    //Wait for user to prompt for calibration
    if(soft.check() == 1) {
      xMax.findLimit();
    }
    return 0;
  } else if( yMin.limitSet == 0) {
    //oled.doFullScreen(4, 1500); //change this to prompt for find limit graphic
    //Wait for user to prompt for calibration
    if(soft.check() == 1) {
      yMin.findLimit();
    }
    return 0;
  } else if( yMax.limitSet == 0) {
    //oled.doFullScreen(5, 1500); //change this to prompt for find limit graphic
    //Wait for user to prompt for calibration
    if(soft.check() == 1) {
      yMax.findLimit();
    }
    return 0;
  } else {
    return 1;
  }
}


void setup(void) {
  //Start I2C communication
  Wire.begin(D3,D4);
  Wire.setClock(400000);

  //Start the port expander
  mcp.begin();

  //Initialize the stepper motors
  motors[0] = new Stepper(0, 1, 0);
  motors[1] = new Stepper(2, 3, 1);

  //Print WiFi debug stuff
  Serial.setDebugOutput(true);

  //Start serial connection
  Serial.begin(115200);

  //Start the I2C PWM board
  pwm.begin();
  pwm.setPWMFreq(1000);

  //Start the OLED display and display our project logo
  //display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  //display.clearDisplay();
  //display.drawBitmap(1, 1,  new_logo, 128, 64, 1);
  //display.display();

  joinWiFi();

  if (!ioClient.connect(const_cast<char*>(myhost), httpPort) ) {
    Serial.println("connection failed");
  }
  if (ioClient.connected())
  {
    ioClient.send("connection", "message", "Connected !!!!");
  }


  delay(2000);
}

unsigned int statusCheckMillis = 0;

void loop(void) {

  unsigned int currentMillis = millis();

  if((unsigned long)(currentMillis - statusCheckMillis) > 2500) {
    command.wsDisplay();
    statusCheckMillis = currentMillis;
  }

  //Check critical stops and calibration state. Don't run any code until we know stops are cleared and calibration is good.
  if( !passedCriticalStops() ) {
    //yield(); //Let the MCU perform background tasks.
  } else if ( !foundLimits()) {
  //Get a command. If none ready, wait a few seconds and try again.
  } else if ( !command.fetchCommand() ) {
    //yield();
  } else {
    //Run the command as long as it's active
    //This takes very little time (u-seconds) and then frees up to detect button presses.
    command.doCommand();
  }


} //End loop
