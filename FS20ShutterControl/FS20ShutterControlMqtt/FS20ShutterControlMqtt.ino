#include <Ethernet.h>
#include <Dhcp.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

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

// MQTT broker IP address
const char* mqttServer     = "192.168.178.27";
const char* mqttClientName = "FS20Control";

// MQTT topics
const char* topicShutterControlLeft          = "livingroom/shutter/left";
const char* topicShutterControlRight         = "livingroom/shutter/right";

// Update these with values suitable for your network.
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };


// global variables
EthernetClient ethClient;
PubSubClient client(ethClient);
SoftwareSerial mySerial(9, 2);                       // labeled on shield: TXD, RXD



void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if(strcmp(topic,topicShutterControlLeft)==0) {
    if(strncmp(payload,"0",length)==0) {
      Serial.println("opening shutter left");
      mySerial.println(CMD_SHUTTER_OPEN_EGL);
    }
    else if(strncmp(payload,"100",length)==0) {
      Serial.println("closing shutter left");
      mySerial.println(CMD_SHUTTER_CLOSE_EGL);
    }
    else {
      Serial.println("Invalid payload");
    }
  }

  if(strcmp(topic,topicShutterControlRight)==0) {
    if(strncmp(payload,"0",length)==0) {
      Serial.println("opening shutter right");
      mySerial.println(CMD_SHUTTER_OPEN_EGR);
    }
    else if(strncmp(payload,"100",length)==0) {
      Serial.println("closing shutter right");
      mySerial.println(CMD_SHUTTER_CLOSE_EGR);
    }
    else {
      Serial.println("Invalid payload");
    }
  }  
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(mqttClientName)) {
      Serial.println("connected");
      
      // resubscribe
      client.subscribe(topicShutterControlLeft);
      client.subscribe(topicShutterControlRight);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void setup() {
  // initialize serial debug connection
  Serial.begin(115200);
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
    
  // enable reception on CSM
  mySerial.println("X01");
  
  // set Tx power to +10dBm
  mySerial.println("x04");

  client.setServer(mqttServer, 1883);
  client.setCallback(callback);

  if(Ethernet.begin(mac)) {
    Serial.print( F("DHCP success. IP=") );
    Serial.println(Ethernet.localIP());
  }
  else {
    Serial.println( F("DHCP failed") );
  }
  
  //Ethernet.begin(mac, ip);
  // Allow the hardware to sort itself out
  delay(1500);
  
  Serial.println( F("setup() end") );
}


void loop() {
  static unsigned int counter = 0;
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
    
  // and sleep 1s
  delay(1000);
}
