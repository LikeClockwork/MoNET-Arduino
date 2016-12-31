#include <Wire.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>
#include "Adafruit_MCP23017.h"
#include "bitmaps.h"

//Set up our display, pwm, and port expander boards
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Adafruit_MCP23017 mcp;
Adafruit_SSD1306 display(D0);

//Timekeeping for button presses

//unsigned long currentMillis = 0;
unsigned long previousMillis = 0;

class Button
{
  unsigned int btnPin; //What pin are we connected to?
  boolean isMomentary; //Is this button momentary or always on?
  byte btnType; // 0 = pause, 1 = e-stop, 3 = soft
  int debounce; //Debounce delay .. how long do you have to hold the button down for

  boolean status; //Is the button's "status" active or not . . . are we paused/stopped/whatever
  boolean buttonPressed; // 3 states: 0 = not pressed, 1 = pressed and not checked, 2 = pressed and checked
  unsigned long checkMillis; //Stores last time button was checked

  public:
  Button(unsigned int pin, boolean momentary, byte type, int dbDelay) {
    btnPin = pin;
    isMomentary = momentary;
    btnType = type;
    debounce = dbDelay;

    pinMode(btnPin,  INPUT_PULLUP);
    checkMillis = 0;
  }

  boolean state (void) {
      return digitalRead(btnPin);
  }

  void changeStatus(void) {
    if(isMomentary == 1) {
      //if momentary, toggle status
      status = status == 0 ? 1 : 0;
    } else {
      //if latching, status is always active if button is down.
      status = 1;
    }
  }

  void check() {
      unsigned long currentMillis = millis();

      //Is button pressed (gone to ground)
      if( state() == 0 ) {
        //Have we read it as pressed yet?
        if(buttonPressed == 0 ) {
          //If not mark it as pressed
          checkMillis = millis();
          buttonPressed = 1;
        } else if ( buttonPressed == 1 ) {
          changeStatus();
          buttonPressed = 3;
        }

      }
  }

};

/*
//Pause contition.
//Pause is a mommentary button that toggles off and on.
#define pausePin    9
#define pauseStatus (digitalRead(9))

boolean paused = false;
byte pausePressed = 0; // 3 states: 0 = not pressed, 1 = pressed and not checked, 2 = pressed and checked
unsigned long pauseCheck = 0; //button debounce timer

//E-stop condition
//As long as this is pressed/down/closed stop condition exists
#define eStopPin    10
#define eStopStatus (digitalRead(10))

boolean stopped = false;
byte eStopPressed = 0; // 3 states: 0 = not pressed, 1 = pressed and not checked, 2 = pressed and checked
unsigned long eStopCheck = 0; //button debounce timer

//Animations for stop and pause conditions
byte stopFrame = 1;
byte pauseFrame = 1;
unsigned long pauseMillis = 0;

void pauseAnimations(void) {
  if(stopped == true && (unsigned long)(currentMillis - pauseMillis) >= 1500 ) {
    switch(stopFrame){
      case 1:
        display.clearDisplay();
        display.drawBitmap(1, 1,  estop_frame1, 128, 64, 1);
        display.display();
        stopFrame = 2;
        pauseMillis = currentMillis;
        break;
      case 2:
        display.clearDisplay();
        display.drawBitmap(1, 1,  estop_frame2, 128, 64, 1);
        display.display();
        stopFrame = 1;
        pauseMillis = currentMillis;
        break;
      }
    } else if(paused == true && (unsigned long)(currentMillis - pauseMillis) >= 1500 ) {
      switch(pauseFrame){
        case 1:
          display.clearDisplay();
          display.drawBitmap(1, 1,  pause_frame1, 128, 64, 1);
          display.display();
          pauseFrame = 2;
          pauseMillis = currentMillis;
          break;
        case 2:
          display.clearDisplay();
          display.drawBitmap(1, 1,  pause_frame2, 128, 64, 1);
          display.display();
          pauseFrame = 3;
          pauseMillis = currentMillis;
          break;
        case 3:
          display.clearDisplay();
          display.drawBitmap(1, 1,  pause_frame3, 128, 64, 1);
          display.display();
          pauseFrame = 1;
          pauseMillis = currentMillis;
          break;
        }
  } else if (stopped == false && paused == false ){
    display.clearDisplay();
    display.display();
  }
}


//Check to see if pause is pressed and, if so, toggle pause status.
void checkPausePress (void) {
  //If button is down and it wasn't previously pressed, record the press and current time.
  if (pauseStatus == 0) {
    if (pausePressed == 0) {
      pauseCheck = currentMillis;
      pausePressed = 1;
    //If the button is pressed, and it was previously pressed and debounce timer is done, toggle pause status.
    } else if(pausePressed == 1 && (unsigned long)(currentMillis - pauseCheck) >= 50 ) {
        if (stopped == false) {
          Serial.print("Paused button pressed. Machine now ");
          if ( paused == true ) {
            paused = false;
            Serial.println("unpaused.");
            pauseFrame = 1;
          } else {
            paused = true;
            Serial.println("paused.");
          }

        } else {
          Serial.println("eStop activated. Can't unpause.");
        }
        pausePressed = 3;
    }
  } else {
    pausePressed = 0;
  }
}

void checkEStopPress (void) {
  //If button is down and it wasn't previously pressed, record the press and current time.
  if (eStopStatus == 0) {
    if (eStopPressed == 0) {
      eStopCheck = currentMillis;
      eStopPressed = 1;
    //If the button is pressed, and it was previously pressed and debounce timer is done, toggle pause status.
    } else if(eStopPressed == 1 && (unsigned long)(currentMillis - eStopCheck) >= 50 ) {
        Serial.println("Machine e-stopped!");
        stopped = true;
        paused = true;
        eStopPressed = 3;
    }
  } else {
    eStopPressed = 0;
    stopped = false;
    stopFrame = 1;
  }
}

//Soft button
#define softPin D7
#define softStatus (digitalRead(D7))

//Autoposition and maintain position
int foundXMin = 0; //three states, 0 = not found, waiting for go signal, 1 = not found and looking, 3 = found
int foundXMax = 0;
int foundYMin = 0;
int foundYMax = 0;

byte softPressed = 0; // 3 states: 0 = not pressed, 1 = pressed and not checked, 2 = pressed and checked
unsigned long softCheck = 0; //button debounce timer
byte softAction = 0;
/* Soft Actions
  0 = xMin
  1 = xMax
  2 = yMin
  3 = yMax
  4 = reset!



void checkSoftPress (void) {
  //If button is down and it wasn't previously pressed, record the press and current time.
  if (softStatus == 0) {
    if (softPressed == 0) {
      softCheck = currentMillis;
      softPressed = 1;
    //If the button is pressed, and it was previously pressed and debounce timer is done, toggle pause status.
  } else if(softPressed == 1 && (unsigned long)(currentMillis - softCheck) >= 50 ) {
        if (softAction == 0) {
          foundXMin = 1;
        } else if (softAction == 1) {
          foundXMax = 1;
        } else if (softAction == 2) {
          foundYMin = 1;
        } else if ( softAction == 3) {
          foundYMax = 1;
        } else if ( softAction == 4) {
          // Do something to reset
        }
    }
  } else {
    softPressed = 0;
  }
}
*/

//Define our stepper commands
#define xGo     (mcp.digitalWrite(0,HIGH))
#define xStop   (mcp.digitalWrite(0,LOW))
#define Right    (mcp.digitalWrite(1,HIGH))
#define Left     (mcp.digitalWrite(1,LOW))
#define yGo     (mcp.digitalWrite(2,HIGH))
#define yStop   (mcp.digitalWrite(2,LOW))
#define Up    (mcp.digitalWrite(3,HIGH))
#define Down     (mcp.digitalWrite(3,LOW))

//Stepper limit switch statuses. These are normally on (1)
#define xMaxPin D1
#define xMinPin D2
#define yMaxPin D5
#define yMinPin D6

#define xMaxStatus (digitalRead(D1))
#define xMinStatus (digitalRead(D2))
#define yMaxStatus (digitalRead(D5))
#define yMinStatus (digitalRead(D6))

#define offsetLimit 20 //Steps to back off of limiters

unsigned int xMax = 4294967295;
unsigned int yMax = 4294967295;

unsigned int xPos = 0; //current x position in steps
unsigned int yPos = 0; //current y position in steps

/*
void checkLimits(void) {
  if( xMaxStatus == 1 || xMinStatus ==1 || yMaxStatus == 1 || yMinStatus == 1 ) {
      paused == true;
  }
}
*/

//Stepper functions
void findXMin(void) {
  //If limit hasn't been found, keep looking for it!
  if(xMinStatus == 0) {
    Left;
    xGo;
    delayMicroseconds(100);
    xStop;
  } else {
  //If limit found, back off a bit.
    Right;
    for(int i = 0; i < offsetLimit; i++) {
      xGo;
      delayMicroseconds(100);
      xStop;
      yield();
    }
    foundXMin = 3;
    xPos = 0;
  }
}

void findXMax(void) {
  //If limit hasn't been found, keep looking for it!
  if(xMaxStatus == 0) {
    Right;
    xGo;
    delayMicroseconds(100);
    xStop;
    xPos++;
  } else {
  //If limit found, back off a bit and save the value.
    Left;
    for(int i = 0; i < offsetLimit; i++) {
      xGo;
      delayMicroseconds(100);
      xStop;
      xPos--;
      yield();
    }
    foundXMax = 3;
    xMax = xPos;
  }
}

void findYMin(void) {
  //If limit hasn't been found, keep looking for it!
  if(yMinStatus == 0) {
    Down;
    yGo;
    delayMicroseconds(100);
    yStop;
  } else {
  //If limit found, back off a bit.
    Up;
    for(int i = 0; i < offsetLimit; i++) {
      yGo;
      delayMicroseconds(100);
      yStop;
      yield();
    }
    foundYMin = 3;
    yPos = 0;
  }
}

void findYMax(void) {
  //If limit hasn't been found, keep looking for it!
  if(yMaxStatus == 0) {
    Up;
    yGo;
    delayMicroseconds(100);
    yStop;
    yPos++;
  } else {
  //If limit found, back off a bit.
    Down;
    for(int i = 0; i < offsetLimit; i++) {
      yGo;
      delayMicroseconds(100);
      yStop;
      yPos--;
      yield();
    }
    foundYMax = 3;
    yMax = yPos;
  }
}

//Get stuff to do
boolean busy = false; //Do I have anything to do?
boolean connectStatus = false; //Can we connect to server?

const char* host = "192.168.1.7"; //Server address www.projectmo.net
const char* hardwareID = "mowpjf38qe"; //Semi-unique hardware ID

unsigned int xDe = 0;
unsigned int y = 0;
byte speed  = 0;
byte red = 0;
byte green = 0;
byte blue = 0;
byte white = 0;
byte black = 0;
byte mix = 0;
byte dispense = 0;

void getCommand () {
  String line;
  Serial.print("Connecting to ");
  Serial.println(host);

  // Use WiFiClient class to create TCP connections
  WiFiClient client;
  const int httpPort = 3000;
  if (!client.connect(host, httpPort)) {
    Serial.println("Connection failed!");
    connectStatus = false;
    return;
  }
  connectStatus = true;
  // We now create a URI for the request
  String url = String("/motor/") + hardwareID;

  // This will send the request to the server
  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
               "Host: " + host + "\r\n" +
               "Accept: */*\r\n" +
               "User-Agent: Mozilla/4.0 (compatible; esp8266 Lua; )\r\n" +
               "Connection: close\r\n\r\n");
  delay(10);

  while(!client.available()){}

  // Read all the lines of the reply from server and print them to Serial
  while(client.available()){
     line = client.readStringUntil('\r');
  }

  Serial.println(line);

  //Server should respond with JSON data that looks like this
  /*
    commad = {
      "x" : 123456, //x destination
      "y" : 123456, //y destination
      "speed" : 234, //speed at which the steppers will move for this command
      "red" : 123, //paint pump rates
      "green" : 123,
      "blue" : 123,
      "white" : 123,
      "black" : 123,
      "mix" : 123,
      "dispense" : 123
  }
  */
  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(line);

  //You have to do some stupid crap where you cast the value into a temp variable for some reason. These are the temp values. Note local scope.

  String tempDir = root["direction"];
  int tempDist = root["steps"];

  Serial.println("closing connection");

  delay(10);
}


//Define paint pins
#define redPin    0
#define greenPin  1
#define bluePin   2
#define whitePin  3
#define blackPin  4
#define mixPin    5
#define sprayPin  6
#define cleanPin  7

//current color values (0 to 255)
byte
  redVal = 0,
  greenVal = 0,
  blueVal = 0,
  whiteVal = 0,
  blackVal = 0;

//Define pump calibrations
#define pumpMin 100.0
#define pumpMax 255.0

void paintPump( int pin, float speed) {
  if (speed > 0) {
    speed = ((pumpMin/pumpMax) * speed) + pumpMin;
  }
  pwm.setPWM(pin ,0, (speed / 255) * 4095);
  Serial.println((speed / 255) * 4095);
}



void setup(void) {
  Wire.begin(D3,D4);
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(1000);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.drawBitmap(1, 1,  new_logo, 128, 64, 1);
  display.display();
  delay(1000);
  //Set button pullups
  pinMode(pausePin, INPUT_PULLUP);
  pinMode(eStopPin, INPUT_PULLUP);
  pinMode(xMinPin, INPUT_PULLUP);
  pinMode(xMaxPin, INPUT_PULLUP);
  pinMode(yMinPin, INPUT_PULLUP);
  pinMode(yMaxPin, INPUT_PULLUP);
  pinMode(softPin, INPUT_PULLUP);
}



int counter = 10000;

void loop(void) {
  //Record current time since live


/*
  currentMillis = millis();

  checkEStopPress();
  checkPausePress();
  pauseAnimations();

  //Check for stop and pause conditions before doing anything serious!
  if( stopped == true || paused == true ) {
    //Check motor calibrations and find zeros
    if( foundXMin != 3 && foundXMax != 3 && foundYMin != 3 && foundYMax != 3) {
      if( foundXMin == 0) {
        softAction = 0;
        checkSoftPress();
      } else if( foundXMin == 1) {
        findXMin();
      } else if( foundXMax == 0) {
        softAction = 1;
        checkSoftPress();
      }  else if( foundXMax == 1) {
        findXMax();
      } else if( foundYMin == 0) {
        softAction = 2;
        checkSoftPress();
      }  else if( foundYMin == 1) {
        findYMin();
      } else if( foundYMax == 0) {
        softAction = 3;
        checkSoftPress();
      }  else if( foundYMax == 1) {
        findYMax();
      }
    } else {
      //Actually do stuff!
*/


    }
  }
}
