#include "conf.h"
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

//Set up our display, pwm, and port expander boards
WiFiClient client;
SoftwareSerial swSer(D6, D5, false, 256);

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
        yield();
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
  //Serial.print("Connecting to ");
  //Serial.println(myhost);
  //Bail out and try again if the connection fails.
  if (!client.connect(myhost, httpPort)) {
    Serial.println("Connection failed!");
    yield();
    return "";
  }
  //Sent the request to the server. This will hit an endpoint which will reply with the next command.
  //Serial.println(params);
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
  delay(10);
  Serial.println(response);
  return response;
}

/*
Status codes
0 = idle
1 = executing command
2 = paused
3 = eStopped
4 = prompt for xMin
5 = prompt for xMax
6 = prompt for yMin
7 = prompt for yMax
8 = finding for xMin
9 = finding for xMax
10 = finding for yMin
11 = finding for yMax
*/


class Status
{
  public:
    unsigned long commandId = 0;
    unsigned long xMax = 0;
    unsigned long yMax = 0;
    unsigned long xPos = 0;
    unsigned long yPos = 0;
    int signal = 0;
    unsigned int statusCode = 0;

  public:
  Status(void) {};

};

Status status;

String createReqString( int actionCode, int reqCount = 1 ) {

  /* actions:
    1: get new command
    2: finish command, get new command.
    3: status update
  */

  String action = "";

  if(actionCode == 1) {
    action = "fetch";
  } else if (actionCode == 2) {
    action = "complete";
  } else {
    action = "status";
  }

  status.signal = WiFi.RSSI();

  String request = "."
    + action + "."
    + (String)status.commandId + "."
    + (String)status.xPos + "."
    + (String)status.yPos + "."
    + (String)status.xMax + "."
    + (String)status.yMax + "."
    + (String)status.signal + "."
    + (String)status.statusCode+ "."
    + (String)reqCount;

    return request;
}


//Get a command string from the server;
String getCommand (int reqCount = 1) {


  String response = "";

  if (status.commandId == 0 ) {
    response = contactServer( createReqString(1 , reqCount) );
  } else {
    response = contactServer( createReqString(2, reqCount) );
  }

  return response;
}

class CommandString
{
  public:
  unsigned long commandId = 0;
  String command = "";

  public:
  CommandString(void) {};
};

CommandString* command[4];

//Get the next command by ID
String getNextCommand () {
  unsigned long tempCommandId = 4294967295;
  String tempCommandString = "";
  int deleteMe = 0;
  boolean found = 0;
  for( int i = 0; i < 4; i++) {
    if (command[i] -> commandId == status.commandId) {
      command[i] -> commandId == 0;
      command[i] -> command == "";
    }


    //Run through commands, get the lowest non-zero command id
    if (command[i] -> commandId != 0 && command[i] -> commandId < tempCommandId) {
      tempCommandId = command[i] -> commandId;
      tempCommandString = command[i] -> command;
      status.commandId = command[i] -> commandId;
      deleteMe = i;
      found = 1;
    }

  }

  if( found ) {
    command[deleteMe] -> commandId == 0;
    command[deleteMe] -> command == "";
  }

  Serial.print("Next command is: ");
  Serial.println(tempCommandString);
  status.commandId = tempCommandId;
  swSer.println(tempCommandString);
  return tempCommandString;
}

void fillCommands (void) {

  int cmdCount = 0;

  for( int i = 0; i < 4; i++) {
    //Run through commands, if any of them are blank/finished, load them up.
    if (command[i] -> commandId == 0 && command[i] -> command == "" ) {
      cmdCount++;
    }
  }

  if (cmdCount == 0) return;

  char response[800] = "";
  String holding = getCommand(cmdCount);
  holding.toCharArray(response, 800);


  //Parse out the JSON and the remap it to a comma delimited string
  DynamicJsonBuffer jsonOut;
  JsonArray& commands = jsonOut.parseArray(response);
  if (!commands.success()) {
     Serial.println("parseArray() failed");
     return;
  }

  int el = 0;

  for( int j = 0; j < 4; j++) {
    //Run through commands, if any of them are blank/finished, load them up.
    if (command[j] -> command == "" ) {
      Serial.print("Loading ");
      String cidTemp = commands[el]["cid"];
      Serial.print(cidTemp);
        Serial.print(" to ");
        Serial.print(j);



      String cr = commands[el]["cr"];
      String cid = commands[el]["cid"];
      String x = commands[el]["x"];
      String y = commands[el]["y"];
      String s = commands[el]["s"];
      String r = commands[el]["r"];
      String g = commands[el]["g"];
      String b = commands[el]["b"];
      String k = commands[el]["k"];
      String w = commands[el]["w"];
      String m = commands[el]["m"];
      String d = commands[el]["d"];
      String cl = commands[el]["cl"];
      String pm = commands[el]["pm"];

      unsigned int numbId = commands[el]["cid"];

      if(!cid) return;

      status.commandId = numbId;
      command[j] -> commandId = numbId;
      command[j] -> command = String("CMD:")
        + cr + ","
        + cid + ","
        + x + ","
        + y + ","
        + s + ","
        + r + ","
        + g + ","
        + b + ","
        + k + ","
        + w + ","
        + m + ","
        + d + ","
        + cl + ","
        + pm;
        Serial.println(command[j] -> command);
        el++;
    }
    yield();
  }

  return;

}

void emptyCommands (void) {
  Serial.println("Purging command buffer:");
  for( int i = 0; i < 4; i++) {
    //Run through commands, if any of them are blank/finished, load them up.
      command[i] -> command = "";
      command[i] -> commandId = 0 ;
    yield();
  }
}

//find command to finish, clear it out, update it
void completeCommand (unsigned long prevId) {
  Serial.print("Finishing command ");
  Serial.println(prevId);
  //Find and delete the finished command.
  for( int i = 0; i < 4; i++) {
    if (command[i] -> commandId == prevId) {
      Serial.print("New command loaded to buffer: ");
      command[i] -> command = "";
      command[i] -> commandId = 0 ;
      break;
    }

    //contactServer(createReqString(2));
    yield();
  }

}

String encryptionTypeStr(uint8_t authmode) {
    switch(authmode) {
        case ENC_TYPE_NONE:
            return "NONE";
        case ENC_TYPE_WEP:
            return "WEP";
        case ENC_TYPE_TKIP:
            return "TKIP";
        case ENC_TYPE_CCMP:
            return "CCMP";
        case ENC_TYPE_AUTO:
            return "AUTO";
        default:
            return "?";
    }
}


void setup(void) {

  //Start serial connection
  Serial.begin(115200);
  swSer.begin(57600);

  //Start the I2C PWM board


  delay(500);

  joinWiFi();

  delay(500);

  command[0] = new CommandString;
  command[1] = new CommandString;
  command[2] = new CommandString;
  command[3] = new CommandString;

  Serial.println("scan start");

  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks(false,true);
  Serial.println("scan done");
  if (n == 0)
    Serial.println("no networks found");
  else
  {

    // sort by RSSI
    int indices[n];
    for (int i = 0; i < n; i++) {
      indices[i] = i;
    }
    for (int i = 0; i < n; i++) {
      for (int j = i + 1; j < n; j++) {
        if (WiFi.RSSI(indices[j]) > WiFi.RSSI(indices[i])) {
          //int temp = indices[j];
          //indices[j] = indices[i];
          //indices[i] = temp;
          std::swap(indices[i], indices[j]);
        }
      }
    }

    Serial.print(n);
    Serial.println(" networks found");

    Serial.println("00: (RSSI)[BSSID][hidden] SSID [channel] [encryption]");
    for (int i = 0; i < n; ++i)
    {
      // Print SSID and RSSI for each network found
      // Serial.print(i + 1);
      Serial.printf("%02d", i + 1);
      Serial.print(":");

      Serial.print(" (");
      Serial.print(WiFi.RSSI(indices[i]));
      Serial.print(")");

      Serial.print(" [");
      Serial.print(WiFi.BSSIDstr(indices[i]));
      Serial.print("]");

      Serial.print(" [");
      Serial.print((String) WiFi.isHidden(indices[i]));
      Serial.print("]");

      Serial.print(" " + WiFi.SSID(indices[i]));
      // Serial.print((WiFi.encryptionType(indices[i]) == ENC_TYPE_NONE)?" ":"*");

      Serial.print(" [");
      Serial.printf("%02d",(int)WiFi.channel(indices[i]));
      Serial.print("]");

      Serial.print(" [");
      Serial.print((String) encryptionTypeStr(WiFi.encryptionType(indices[i])));
      Serial.print("]");

      Serial.println();
      delay(10);
    }
  }
  Serial.println("");

  delay(500);


  joinWiFi();

  delay(500);


}



unsigned long statusCheckMillis = 0;

boolean direction = 0;

int counter = 500;

void loop(void) {


  //If incoming bytes on soft serial . . .
  if( swSer.available()) {
    Serial.print("Command 1: ");
    Serial.println(command[0] -> command);
    Serial.print("Command 2: ");
    Serial.println(command[1] -> command);
    Serial.print("Command 3: ");
    Serial.println(command[2] -> command);
    Serial.print("Command 4: ");
    Serial.println(command[3] -> command);
    //Read to the end of the line
    String incoming = swSer.readStringUntil('\n');
    StaticJsonBuffer<200> jsonIn;
    JsonObject& inc = jsonIn.parseObject(incoming);

    inc.printTo(Serial);
    Serial.println();

    String message = inc["m"];
    unsigned int code = inc["c"] ? inc["c"] : 0 ;
    unsigned long finishCommand = inc["f"] ? inc["f"] : 0;

    if( code > 0 ) {
      //Serial.print("Status code recieved: ");
      //Serial.println(code);
      status.statusCode = code;
      //do something to update status on web
      status.xPos = (unsigned long)inc["x"];
      status.yPos = (unsigned long)inc["y"];
      status.xMax = (unsigned long)inc["xl"];
      status.yMax = (unsigned long)inc["yl"];
      //Send status if not waiting for a command.

    if ( status.statusCode >= 5 && status.statusCode <= 12) {
      //purge commmands
      emptyCommands();
    } else {
      fillCommands();
    }


    if( finishCommand > 0 ) {
          status.commandId = finishCommand;
          completeCommand(finishCommand);
      }

      if( code == 17 || code == 18 ) {
          getNextCommand();
      }


      if( code != 17 && code != 18) {
        //contactServer( createReqString(3) );
      }
    }
    //Check if any of the commands are empty and fill them
    //fillCommands();
  }

} //End loop
