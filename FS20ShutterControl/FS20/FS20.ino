#include <EEPROM.h>

#include <Dhcp.h>
#include <Dns.h>
#include <Ethernet.h>
#include <EthernetClient.h>
#include <EthernetServer.h>
#include <EthernetUdp.h>

#include <TinyTime.h>

#include <SPI.h>
#include <Dhcp.h>
#include <Dns.h>
#include <Ethernet.h>
#include <EthernetClient.h>
#include <EthernetServer.h>
#include <EthernetUdp.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <TinyTime.h>
//#include <util.h>

#include <avr/pgmspace.h> // for progmem

/**
 * port overview:
 *  0 : Serial RX
 *  1 : Serial TX
 *  2 : 
 *  3 : 
 *  4 : 
 *  5 : 
 *  6 :
 *  7 : 
 *  8 :
 *  9 :
 * 10 : SPI SS
 * 11 : SPI MOSI
 * 12 : SPI MISO
 * 13 : SPI SCK
*/


/*
  Software serial multiple serial test
 
 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.
 
 The circuit (default as shipped): 
 * RX is digital pin 10 (connect to TXD of CSM device)
 * TX is digital pin 2 (connect to RXD of CSM device)
 
 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts, 
 so only the following can be used for RX: 
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69
 
 Not all pins on the Leonardo support change interrupts, 
 so only the following can be used for RX: 
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).
 
 created back in the mists of time
 modified 28 May 2013
 by Dirk Tostmann
 based on Mikal Hart's example
 
 This example code is in the public domain.
 
 */
 

// definition of constants

// These two pins are only active if jumpers have been placed at the shield
// jumpers are not required for simple operation. They will allow you to restart and/or change CSM firmware on the fly.
const int csm_reset = 7;
const int csm_boot  = 6;
const unsigned int UDP_PORT = 5000;                  // UDP port for remote access

//
// struct to store temperature & humidity sensor measurement result
//
struct ThsMeasurement {
  boolean       isValid;
  unsigned long timeOfDay;
  int           temp;              // in tenth of degrees
  unsigned int  humidity;          // in tenth of percent
  
  // constructor to initialize
  ThsMeasurement() : isValid(false),timeOfDay(0),temp(0),humidity(0) {};
};


/*
 format of FS20 commands: Fhhhhaacc
 hhhh: Hauscode 3627
 aa:   address
 cc:   command (e.g. 00=off,10=on)
*/
const char* CMD_SHUTTER_OPEN_EGL  = "F36270010";
const char* CMD_SHUTTER_OPEN_EGR  = "F36270110";
const char* CMD_SHUTTER_CLOSE_EGL = "F36270000";
const char* CMD_SHUTTER_CLOSE_EGR = "F36270100";
const char* CMD_GARAGE_ON         = "F36270201";
const char* CMD_GARAGE_OFF        = "F36270200";
const char* CMD_DUMMY             = "F36271000";



// global variables
Time time(2);                                        // time base with 2 events
EthernetUDP udpServer;                               // UDP server for remote network access
SoftwareSerial mySerial(9, 2);                       // labeled on shield: TXD, RXD

// dummy IP address of bedroom Arduino
// used for workaround to solve problems with WLAN repeater
IPAddress dummyIp(192, 168, 178, 33);

// variables for min, max and actual temperature
ThsMeasurement tempMin,
               tempMax,
               tempAct;

// event IDs
unsigned int eventIdShutterOpenEg,
             eventIdShutterCloseEg;

// processes and stores a temp measurement
void processThsMeasurement(const char* buffer);

// declaration of the command handler functions for remote (UDP) access
bool cmdShutter(unsigned int,char**,char*);    // shutter control
bool cmdAutomate(unsigned int,char**,char*);   // automate control
bool cmdTime(unsigned int,char**,char*);       // time command
bool cmdThs(unsigned int,char**,char*);        // ths command
bool cmdGarage(unsigned int,char**,char*);     // garage command

// declaration of the event handler functions for time based events
void shutterOpenEG();
void shutterCloseEG();

// this struct maps remote command names to function pointers which implement the execution
struct UdpCommand {
  char* cmd;
  bool (*pFunction)(unsigned int,char**,char*);
};

// available UDP remote commands
UdpCommand udpCommands[] = { {"shutter",&cmdShutter},
                             {"auto",   &cmdAutomate},
                             {"time",   &cmdTime},
                             {"ths",    &cmdThs},
                             {"garage", &cmdGarage} };


void setup() {
  // initialize serial debug connection
  Serial.begin(19200);
  Serial.println( F("Arduino connected") );
  
  pinMode(csm_reset, OUTPUT);     
  pinMode(csm_boot,  OUTPUT);     
  digitalWrite(csm_boot, HIGH);   // this wont select the bootloader! 
  digitalWrite(csm_reset, LOW);   // this will reset the CSM 
  delay(200);                     // wait for a while

  digitalWrite(csm_reset, HIGH);  // get CSM out of RESET, start it ...
  delay(1000);                    // wait for a while

  // set the data rate for the SoftwareSerial port towards CSM
  mySerial.begin(38400);
    
  // initialize ethernet server
  byte mac[] = { 0x90, 0xA2, 0xDA, 0x0D, 0x75, 0xF5 }; 
  if(Ethernet.begin(mac)) {
    Serial.print( F("DHCP success. IP=") );
    Serial.println(Ethernet.localIP());
  }
  else {
    Serial.println( F("DHCP failed") );
  }
  
  // initialize timebase
  time.init();
  time.sync();
  Serial.println( F("time initialized") );
  
  // add events for automatic opeen/close of shutters
  eventIdShutterOpenEg  = time.addEvent(&shutterOpenEG);
  eventIdShutterCloseEg = time.addEvent(&shutterCloseEG);
  time.setEventFireOnce(eventIdShutterOpenEg,false);
  time.setEventFireOnce(eventIdShutterCloseEg,false);
  
  // read event EEPROM data andsync again to get updated times of relative events
  time.readEeprom();
  time.sync();
  
  // start to listen for UDP packets
  udpServer.begin(UDP_PORT);
  
  // enable reception on CSM
  mySerial.println("X01");
  
  // set Tx power to +10dBm
  mySerial.println("x04");
  
  Serial.println( F("setup() end") );
}


void loop() {
  static unsigned int counter = 0;
  
  // ASH 2200 uses the S300 protocol, returning a character string:
  // Kabcdefgh
  // MSB of a: sign, if set, temp is negative, remaining bits are sensor address.
  // b: unknown
  // temp     = 10*f + c + 0.1*d   in deg celcius
  // humidity = 10*g + h + 0.1*e   in %
  
  if (mySerial.available()) {
    if(mySerial.read()=='K') {
      const int MAX_BYTES = 8;
      char      buffer[MAX_BYTES+1];
      int       byte = 0;
      while(mySerial.available() && byte<MAX_BYTES) {
        buffer[byte++] = mySerial.read();
      }
      buffer[byte] = '\0';
      //Serial.print("received measurement: ");Serial.println(buffer);
  
      processThsMeasurement(buffer);
    }
    else {
      // discard
      while(mySerial.available()) {
        mySerial.read();
      }
    }
  }
  
  // if there's data available, read a packet and parse the command
  int packetSize = udpServer.parsePacket();
  if(packetSize)
  {
    processUdpCmd(packetSize);
  }
  
  // to improve responsiveness for UDP access, check for new events
  // only around every 1s
  if(++counter >= 100) {
    // update time
    time.update();
  
    // check for events to be fired (alarm clock)
    time.checkEvents();
    
    counter = 0;
  }
  
  // and sleep 10ms
  delay(10);
}


//
// process command received over UDP connection
// packetSize: packet size
//
void processUdpCmd(unsigned int packetSize) {
  Serial.print( F("Received UDP packet from ") );
  Serial.print(udpServer.remoteIP());
  Serial.print( F(", port ") );
  Serial.println(udpServer.remotePort());

  // read the packet into packetBufffer
  char packetBuffer[UDP_TX_PACKET_MAX_SIZE],           // buffer to hold incoming packet
       answer[UDP_TX_PACKET_MAX_SIZE];
       
  udpServer.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);
  packetBuffer[packetSize] = '\0';
  Serial.print( F("Content: ") );
  Serial.println(packetBuffer);
  
  // workaround for WLAN repeater issue
  // check for command 'dummy' and send UDP packet to dummy IP address
  if(strcmp_P(packetBuffer,PSTR("dummy") )==0) {
    Serial.println( F("received dummy packet") );
    udpServer.beginPacket(dummyIp, udpServer.remotePort());
    udpServer.write(packetBuffer);
    udpServer.endPacket();
    
    udpServer.beginPacket(udpServer.remoteIP(), udpServer.remotePort());
    udpServer.write(packetBuffer);
    udpServer.endPacket();
  }
  else {
    // parse command
    if( parseCmd(packetBuffer,answer) ) {
      // command could be successfully parsed
      strcpy_P(packetBuffer,PSTR("OK "));
    }
    else {
      strcpy_P(packetBuffer,PSTR("ERROR "));
    }
    strcat(packetBuffer,answer);
      
    udpServer.beginPacket(udpServer.remoteIP(), udpServer.remotePort());
    udpServer.write(packetBuffer);
    udpServer.endPacket();
  }
}


/**
 * parses a command string and executes the appropriate handler function
 * returns true on success
 * answer can store up to 17 characters of answer text (plus 0)
 */
bool parseCmd(char* buffer,char* answer) {
  // convert to lowercase
  for(unsigned int i=0 ; i<strlen(buffer) ; i++) {
    buffer[i] = tolower(buffer[i]);
  }

  // get number of available commands
  const unsigned int cmdCount = sizeof(udpCommands)/sizeof(udpCommands[0]);

  // split into parameter and store in separate array as string and int
  unsigned int tokenCount = 0;        // counted tokens in buffer
  const unsigned int MAX_PARAMS = 6;  // allow max. 5 arguments (incl command)
  char* argv[MAX_PARAMS];             // stores char* pointers to each argument
  unsigned int cmd =  cmdCount;       // means no command found

  char* pToken = strtok(buffer," ");
  while(pToken!=NULL) {
    if(tokenCount==0) {
      // check for command
      for(cmd=0 ; cmd<cmdCount ; cmd++) {
        if(strcmp(udpCommands[cmd].cmd,pToken)==0) {
          // command found
          break;
        }
      }
    }
    if(tokenCount>0 && tokenCount<=MAX_PARAMS) {
      argv[tokenCount-1] = pToken;
    }
    pToken = strtok(NULL," ");
    tokenCount++;
  }

  bool retval;
  if(cmd<cmdCount) {
    // valid command found. Call handling function
    answer[0] = '\0';
    retval = udpCommands[cmd].pFunction(tokenCount-1,argv,answer);
  }
  else {
    // no valid command received
    strcpy_P(answer,PSTR("invalid command"));
    retval = false;
  }

  return retval;
}


/**
 * shutter command
 */
bool cmdShutter(unsigned int argc,char** argv,char *answer) {
  if(argc!=2) {
    strcpy_P(answer,PSTR("shutter <cmd> <t>"));
    return false;
  }

  if(strcmp_P(argv[0],PSTR("open"))==0) {
    if(strcmp_P(argv[1],PSTR("egl"))==0) {
      mySerial.println(CMD_SHUTTER_OPEN_EGL); // left
      return true;
    }
    else if(strcmp_P(argv[1],PSTR("egr"))==0) {
      mySerial.println(CMD_SHUTTER_OPEN_EGR); // right
      return true;
    }
    else {
      strcpy_P(answer,PSTR("unknown type"));
    }
  }
  else if(strcmp_P(argv[0],PSTR("close"))==0) {
    if(strcmp_P(argv[1],PSTR("egl"))==0) {
      mySerial.println(CMD_SHUTTER_CLOSE_EGL); // left
      return true;
    }
    else if(strcmp_P(argv[1],PSTR("egr"))==0) {
      mySerial.println(CMD_SHUTTER_CLOSE_EGR); // right
      return true;
    }
    else {
      strcpy_P(answer,PSTR("unknown type"));
    }
  }
  else {
    strcpy_P(answer,PSTR("unknown cmd"));
  }
    
  return false;
}

/**
 * automate command, e.g. automate shutteropen eg
 */
bool cmdAutomate(unsigned int argc,char** argv,char *answer) {
  if(argc<3) {
    strcpy(answer,"auto p1 p2 p3");
    return false;
  }
  
  if(strcmp_P(argv[0],PSTR("open"))==0) {
    if(strcmp_P(argv[1],PSTR("eg"))==0) {
      if(strcmp_P(argv[2],PSTR("abs"))==0) {
        time.setEventMode(eventIdShutterOpenEg,Time::ABSOLUTE);
        time.setEventTime(eventIdShutterOpenEg,atol(argv[3]) % 86400);
        return true;
      }
      else if(strcmp_P(argv[2],PSTR("rise"))==0) {
        time.setEventMode(eventIdShutterOpenEg,Time::SUNRISE);
        time.setEventOffset(eventIdShutterOpenEg,atol(argv[3]) % 86400);
        return true;
      }
      else if(strcmp_P(argv[2],PSTR("set"))==0) {
        time.setEventMode(eventIdShutterOpenEg,Time::SUNSET);
        time.setEventOffset(eventIdShutterOpenEg,atol(argv[3]) % 86400);
        return true;
      }
      else if(strcmp_P(argv[2],PSTR("off"))==0) {
        time.setEventMode(eventIdShutterOpenEg,Time::OFF);
        return true;
      }
      else if(strcmp_P(argv[2],PSTR("getmode"))==0) {
        itoa(time.getEventMode(eventIdShutterOpenEg),answer,10);
        return true;
      }
      else if(strcmp_P(argv[2],PSTR("gettime"))==0) {
        ltoa(time.getEventTime(eventIdShutterOpenEg),answer,10);
        return true;
      }
      else if(strcmp_P(argv[2],PSTR("getoffset"))==0) {
        ltoa(time.getEventOffset(eventIdShutterOpenEg),answer,10);
        return true;
      }
    }
  }
  else if(strcmp_P(argv[0],PSTR("close"))==0) {
    if(strcmp_P(argv[1],PSTR("eg"))==0) {
      if(strcmp_P(argv[2],PSTR("abs"))==0) {
        time.setEventMode(eventIdShutterCloseEg,Time::ABSOLUTE);
        time.setEventTime(eventIdShutterCloseEg,atol(argv[3]) % 86400);
        return true;
      }
      else if(strcmp_P(argv[2],PSTR("rise"))==0) {
        time.setEventMode(eventIdShutterCloseEg,Time::SUNRISE);
        time.setEventOffset(eventIdShutterCloseEg,atol(argv[3]) % 86400);
        return true;
      }
      else if(strcmp_P(argv[2],PSTR("set"))==0) {
        time.setEventMode(eventIdShutterCloseEg,Time::SUNSET);
        time.setEventOffset(eventIdShutterCloseEg,atol(argv[3]) % 86400);
        return true;
      }
      else if(strcmp_P(argv[2],PSTR("off"))==0) {
        time.setEventMode(eventIdShutterCloseEg,Time::OFF);
        return true;
      }
      else if(strcmp_P(argv[2],PSTR("getmode"))==0) {
        itoa(time.getEventMode(eventIdShutterCloseEg),answer,10);
        return true;
      }
      else if(strcmp_P(argv[2],PSTR("gettime"))==0) {
        ltoa(time.getEventTime(eventIdShutterCloseEg),answer,10);
        return true;
      }
      else if(strcmp_P(argv[2],PSTR("getoffset"))==0) {
        ltoa(time.getEventOffset(eventIdShutterCloseEg),answer,10);
        return true;
      }
    }
  }
  
  return false;
}


/**
 * command handler for the time command
 * deals with time retrieval from NTP server
 */
bool cmdTime(unsigned int argc,char** argv,char *answer) {
  if(argc!=1) {
    strcpy(answer,"time <command>");
    return false;
  }

  if(strcmp(argv[0],"get")==0) {
    ltoa(time.getSecondsOfDay(),answer,10);
  }
  else if(strcmp(argv[0],"sync")==0) {
    time.update(true);
    strcpy(answer,time.getSyncTimestamp());
  }
  else if(strcmp(argv[0],"lastsync")==0) {
    strcpy(answer,time.getSyncTimestamp());
  }
  else if(strcmp(argv[0],"error")==0) {
    ltoa(time.getSyncError(),answer,10);
  }
  else if(strcmp(argv[0],"sunrise")==0) {
    ltoa(time.getSunriseTime(),answer,10);
  }
  else if(strcmp(argv[0],"sunset")==0) {
    ltoa(time.getSunsetTime(),answer,10);
  }
  else {
    // unknown command
    strcpy(answer,"invalid subcmd");
    return false;
  }

  return true;
}

/**
 * command handler for the temp command
 * deals with temperature measurements
 */
bool cmdThs(unsigned int argc,char** argv,char *answer) {
  if(argc!=2) {
    strcpy(answer,"ths <command>");
    return false;
  }

  if(strcmp(argv[0],"temp")==0) {
    if(strcmp(argv[1],"act")==0) {
      if(tempAct.isValid) {
        // sprintf doesn't work
        itoa(tempAct.temp,answer,10);
      }
    }
    else if(strcmp(argv[1],"min")==0) {
      if(tempMin.isValid) {
        // sprintf doesn't work
        itoa(tempMin.temp,answer,10);
      }
    }
    else if(strcmp(argv[1],"max")==0) {
      if(tempMax.isValid) {
        // sprintf doesn't work
        itoa(tempMax.temp,answer,10);
      }
    }
    else {
      // unknown command
      strcpy(answer,"invalid ths temp cmd");
      return false;
    }
  }
  else if(strcmp(argv[0],"humidity")==0) {
    if(strcmp(argv[1],"act")==0) {
      if(tempAct.isValid) {
        // sprintf doesn't work
        itoa(tempAct.humidity,answer,10);
      }
    }
    else if(strcmp(argv[1],"min")==0) {
      if(tempMin.isValid) {
        // sprintf doesn't work
        itoa(tempMin.humidity,answer,10);
      }
    }
    else if(strcmp(argv[1],"max")==0) {
      if(tempMax.isValid) {
        // sprintf doesn't work
        itoa(tempMax.humidity,answer,10);
      }
    }
    else {
      // unknown command
      strcpy(answer,"invalid ths humidity cmd");
      return false;
    }
  }
  else if(strcmp(argv[0],"time")==0) {
    if(strcmp(argv[1],"act")==0) {
      if(tempAct.isValid) {
        // sprintf didn't work
        ltoa(tempAct.timeOfDay,answer,10);
      }
    }
    else if(strcmp(argv[1],"min")==0) {
      if(tempMin.isValid) {
        // sprintf didn't work
        ltoa(tempMin.timeOfDay,answer,10);
      }
    }
    else if(strcmp(argv[1],"max")==0) {
      if(tempMax.isValid) {
        // sprintf didn't work
        ltoa(tempMax.timeOfDay,answer,10);
      }
    }
    else {
      // unknown command
      strcpy(answer,"invalid ths time cmd");
      return false;
    }
  }
  else {
      // unknown command
      strcpy(answer,"invalid ths cmd");
      return false;
  }

  return true;
}

/**
 * garage command
 */
bool cmdGarage(unsigned int argc,char** argv,char *answer) {
  if(argc!=1) {
    strcpy(answer,"garage <on|off<onoff>");
    return false;
  }
  
  if(strcmp(argv[0],"on")==0) {
    mySerial.println(CMD_GARAGE_ON);
  }
  else if(strcmp(argv[0],"off")==0) {
    mySerial.println(CMD_GARAGE_OFF);
  }
  else if(strcmp(argv[0],"onoff")==0) {
    mySerial.println(CMD_GARAGE_ON);
    delay(2000);
    mySerial.println(CMD_GARAGE_OFF);
  }
  else {
    strcpy(answer,"garage <on|off<onoff>");
    return false;
  }
  
  return true;
}

void shutterOpenEG() {
  mySerial.println(CMD_SHUTTER_OPEN_EGL); // left
  delay(2000);
  mySerial.println(CMD_SHUTTER_OPEN_EGR); // right
}

void shutterCloseEG() {
  mySerial.println(CMD_SHUTTER_CLOSE_EGL); // left
  delay(2000);
  mySerial.println(CMD_SHUTTER_CLOSE_EGR); // right
}

void processThsMeasurement(const char* buffer) {
  static unsigned long lastTime = 86400L;
  
  // ASH 2200 uses S300 protocol:
  // Kabcdefgh   buffer doesn't contain K any more
  // MSB of a: sign, if set, temp is negative, rest is sensor address.
  // b: unknown
  // temp     = 10*f + c + 0.1*d   in deg celcius
  // humidity = 10*g + h + 0.1*e   in %
  
  // temperature and humidity stored in tenth of degrees
  int temp = 100*(buffer[5]-'0') + 10*(buffer[2]-'0') + (buffer[3]-'0');
  if(buffer[0] & 8) {
    temp = -temp;
  }
  unsigned int humidity = 100*(buffer[6]-'0') + 10*(buffer[7]-'0') + (buffer[4]-'0');
  //Serial.print("Temp=");Serial.println(temp); 
  //Serial.print("Humidity=");Serial.println(humidity); 
  
  // get actual time
  unsigned long now = time.getSecondsOfDay();
  if(now<lastTime) {
    // new day
    tempMin.isValid = false;
    tempMax.isValid = false;
    tempAct.isValid = false;
  }
  
  tempAct.isValid   = true;
  tempAct.timeOfDay = now;
  tempAct.temp      = temp;
  tempAct.humidity  = humidity;
  
  if(!tempMin.isValid || temp<tempMin.temp) {
    tempMin = tempAct;
  }
  
  if(!tempMax.isValid || temp>tempMax.temp) {
    tempMax = tempAct;
  }
  
  lastTime = now;
}


