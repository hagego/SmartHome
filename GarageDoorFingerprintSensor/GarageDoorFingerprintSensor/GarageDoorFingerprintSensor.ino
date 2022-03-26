/*
  ESP8266 code to control a garage door triggered by MQTT
  including Fingerprint Sensor R503
  Wires:
  1: red    VDD 3.3V
  2: black  GND
  3: yellow Serial TX => ESP gpio0
  4: brown  Serial RX gpio2
  5: blue   wake-up (unused)
  6: white  VDD 3.3V
  
  */

// mosquitto 1.3.4 speaks MQTT Version 3.1
// change to MQTT_VERSION MQTT_VERSION_3_1_1 after upgrade to 1.3.5 or higher

#define MQTT_VERSION MQTT_VERSION_3_1

#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Adafruit_Fingerprint.h>

#include "WifiInfo.h"

#ifndef WIFI_SSID
#define WIFI_SSID "my_ssid"
#define WIFI_PSK  "my_psk"
#endif

const char* ssid     = WIFI_SSID;
const char* password = WIFI_PSK;

// OTA hostname
const char* OTA_HOSTNAME = "GarageDoorServerWithFingerprint";

// MQTT broker IP address and client name
const char* mqtt_server = "192.168.178.27";
const char* mqttClientName = "GarageDoorServer2";

// MQTT topics
const char* mqttTopicToggleDoor  = "garage/door";
const char* mqttTopicStatus      = "garage/doorServerStatus";

// MQTT command topic. triggers to enroll a (new) fingerprint. Value: fingerprint ID (1...127)
const char* mqttTopicFingerprintEnrollStart   = "garage/fingerprint/enroll/start";
// MQTT publish topic. sends out enroll status
const char* mqttTopicFingerprintEnrollStatus  = "garage/fingerprint/enroll/status";
// MQTT publish topic. sends out status during fingerprint detection
const char* mqttTopicFingerprintDetectStatus  = "garage/fingerprint/detect/status";

// MQTT callback function for subscribed topics
void mqttCallback(char* topic, byte* payload, unsigned int length);

// MQTT ping interval (ms)
const long mqttPingInterval = 3600000;

// fingerprint sensor SW Serial pins
// D1 mini setup
// const int FINGERPRINT_SW_SERIAL_RX = 4;   // gpio4 = D1 mini D2
// const int FINGERPRINT_SW_SERIAL_TX = 5;   // gpio5 = D1 mini D1

// ESP8266 setup
const int FINGERPRINT_SW_SERIAL_RX = 0;   // gpio0, yellow
const int FINGERPRINT_SW_SERIAL_TX = 2;   // gpio2, brown


WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

// SW serial is used to communicate with finderprint sensor
SoftwareSerial mySerial(FINGERPRINT_SW_SERIAL_RX, FINGERPRINT_SW_SERIAL_TX);
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

void sendPing();


/**
 * setup function
 */
void setup() {
  //Serial.begin(115200);
  Serial.begin(9600);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(OTA_HOSTNAME);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // set up MQTT client
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(60);
  if (mqttClient.connect(mqttClientName)) {
      Serial.println("connected to MQTT server");
      mqttClient.publish(mqttTopicStatus, "started",true);

      mqttClient.subscribe(mqttTopicToggleDoor);
      mqttClient.subscribe(mqttTopicFingerprintEnrollStart);
  }
  else {
    Serial.print("MQTT connection failed: ");
    Serial.println(mqttClient.state());
  }

  // setup serial port for fingerprint sensor
  finger.begin(57600);
  delay(5);
  if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Did not find fingerprint sensor :(");
  }

  Serial.println(F("Reading sensor parameters"));
  finger.getParameters();
  Serial.print(F("Status: 0x")); Serial.println(finger.status_reg, HEX);
  Serial.print(F("Sys ID: 0x")); Serial.println(finger.system_id, HEX);
  Serial.print(F("Capacity: ")); Serial.println(finger.capacity);
  Serial.print(F("Security level: ")); Serial.println(finger.security_level);
  Serial.print(F("Device address: ")); Serial.println(finger.device_addr, HEX);
  Serial.print(F("Packet len: ")); Serial.println(finger.packet_len);
  Serial.print(F("Baud rate: ")); Serial.println(finger.baud_rate);

  finger.getTemplateCount();

  if (finger.templateCount == 0) {
    Serial.print("Sensor doesn't contain any fingerprint data.");
  }
  else {
    Serial.print("Sensor contains "); Serial.print(finger.templateCount); Serial.println(" templates");
  }
}

/**
 * loop function
 */
void loop() {
  // ensure we are still connected
  while (WiFi.status() != WL_CONNECTED){
    Serial.println("WIFI disconnected");
    WiFi.begin(ssid, password);
    uint8_t timeout = 8;
    while (timeout && (WiFi.status() != WL_CONNECTED)) {
      timeout--;
      delay(1000);
    }
    if(WiFi.status() == WL_CONNECTED) {
      Serial.println("WIFI reconnected");
    }
  }

  // check for OTA
  ArduinoOTA.handle();
  mqttClient.loop();

  // send MQTT ping message if needed
  sendPing();

  // check for a fingerprint
  getFingerprintID();

  delay(500);
}

////////////////////////////////////////////////////////////////////////////////////////


/**
 * sends a ping message to the MQTT server
 */
void sendPing()
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
 
  // if enough millis have elapsed
  if (currentMillis - previousMillis >= mqttPingInterval)
  {
    previousMillis = currentMillis;
    
    Serial.println("sending ping");

    // check MQTT connection status and reconnect if needed
    if(!mqttClient.connected()){
      Serial.println("MQTT client disconnected");
      mqttClient.connect(mqttClientName);
      uint8_t timeout = 8;
      while (timeout && (!mqttClient.connected())){
        timeout--;
        mqttClient.connect(mqttClientName);
        delay(1000);
      }
      if(mqttClient.connected()){
        Serial.println("MQTT client reconnected");
        mqttClient.publish(mqttTopicStatus, "reconnected",false);
        mqttClient.subscribe(mqttTopicToggleDoor);

        delay(500);
      }
    }
  
    mqttClient.publish(mqttTopicStatus, "ping",false);
  }
}


/**
 * MQTT callback function
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  char payloadString[length+1];
  strncpy(payloadString,(char*)payload,length);
  payloadString[length] = 0;
    
  if(strcmp(topic,mqttTopicToggleDoor)==0) {
    Serial.print("received MQTT command ");Serial.println(mqttTopicToggleDoor);
    if(strcmp(payloadString,"ON")==0 || strcmp(payloadString,"toggle")==0) {
      toggleRelay();
      
      mqttClient.publish(mqttTopicStatus, "toggled");
    }
  }

  if(strcmp(topic,mqttTopicFingerprintEnrollStart)==0) {
    Serial.print("received MQTT command ");Serial.println(mqttTopicFingerprintEnrollStart);
    int id = atoi(payloadString);
    Serial.print("fingerprint ID to enroll: ");Serial.println(id);
    getFingerprintEnroll(id);
  }
}

/**
 * closes and opens the relay to toggle the door
 */
void toggleRelay() {
  byte open[]  = {0xA0, 0x01, 0x00, 0xA1};
  byte close[] = {0xA0, 0x01, 0x01, 0xA2};

  Serial.write(close, sizeof(close));
  delay(500);      
  Serial.write(open, sizeof(open));
  Serial.println("\nrelay toggled");
}

uint8_t getFingerprintEnroll(int id) {
  if(id<1 || id>127) {
    Serial.println("invalid ID");
    mqttClient.publish(mqttTopicFingerprintEnrollStatus, "invalid id",false);

    return 0;
  }
  
  int p = -1;
  Serial.print("Waiting for valid finger to enroll as #"); Serial.println(id);
  char buffer[25];
  sprintf(buffer,"start to enroll #%d",id);
  mqttClient.publish(mqttTopicFingerprintEnrollStatus,buffer,false);

  
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
    case FINGERPRINT_OK:
      //Serial.println("Image taken");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Image taken",false);
      break;
    case FINGERPRINT_NOFINGER:
      //Serial.print(".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      //Serial.println("Communication error");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Communication error",false);
      break;
    case FINGERPRINT_IMAGEFAIL:
      //Serial.println("Imaging error");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"IMaging error",false);
      break;
    default:
      //Serial.println("Unknown error");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Unknown error",false);
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(1);
  switch (p) {
    case FINGERPRINT_OK:
      //Serial.println("Image converted");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Image converted",false);
      break;
    case FINGERPRINT_IMAGEMESS:
      //Serial.println("Image too messy");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Image too messy",false);
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      //Serial.println("Communication error");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Communication error",false);
      return p;
    case FINGERPRINT_FEATUREFAIL:
      //Serial.println("Could not find fingerprint features");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Could not find fingerprint features",false);
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      //Serial.println("Invalid image");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Invalid image",false);
      return p;
    default:
      //Serial.println("Unknown error");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Unknown error",false);
      return p;
  }

  //Serial.println("Remove finger");
  mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Remove Finger",false);
  delay(2000);
  //Serial.println("Place same finger again");
  mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Same Finger again",false);
  
  p = 0;
  while (p != FINGERPRINT_NOFINGER) {
    p = finger.getImage();
  }
  Serial.print("ID "); Serial.println(id);
  p = -1;
  
  
  while (p != FINGERPRINT_OK) {
    p = finger.getImage();
    switch (p) {
    case FINGERPRINT_OK:
      //Serial.println("Image taken");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Image taken",false);
      break;
    case FINGERPRINT_NOFINGER:
      //Serial.print(".");
      break;
    case FINGERPRINT_PACKETRECIEVEERR:
      //Serial.println("Communication error");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Communication error",false);
      break;
    case FINGERPRINT_IMAGEFAIL:
      //Serial.println("Imaging error");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Imaging error",false);
      break;
    default:
      //Serial.println("Unknown error");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Unknown error",false);
      break;
    }
  }

  // OK success!

  p = finger.image2Tz(2);
  switch (p) {
    case FINGERPRINT_OK:
      //Serial.println("Image converted");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Image converted",false);
      break;
    case FINGERPRINT_IMAGEMESS:
      //Serial.println("Image too messy");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Image too messy",false);
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      //Serial.println("Communication error");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Communication error",false);
      return p;
    case FINGERPRINT_FEATUREFAIL:
      //Serial.println("Could not find fingerprint features");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Could not find fingerprint features",false);
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      //Serial.println("Invalid image");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Invalid image",false);
      return p;
    default:
      //Serial.println("Unknown error");
      mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Unknown error",false);
      return p;
  }

  // OK converted!
  Serial.print("Creating model for #");  Serial.println(id);

  p = finger.createModel();
  if (p == FINGERPRINT_OK) {
    //Serial.println("Prints matched!");
    mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Prints matched",false);
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    //Serial.println("Communication error");
    mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Communication error",false);
    return p;
  } else if (p == FINGERPRINT_ENROLLMISMATCH) {
    //Serial.println("Fingerprints did not match");
    mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Fingerprints did not match",false);
    return p;
  } else {
    //Serial.println("Unknown error");
    mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Unknown error",false);
    return p;
  }

  //Serial.print("ID "); Serial.println(id);
  p = finger.storeModel(id);
  if (p == FINGERPRINT_OK) {
    //Serial.println("Stored!");
    mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Stored!",false);
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    //Serial.println("Communication error");
    mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Communication error",false);
    return p;
  } else if (p == FINGERPRINT_BADLOCATION) {
    //Serial.println("Could not store in that location");
    mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Could not sore in that location",false);
    return p;
  } else if (p == FINGERPRINT_FLASHERR) {
    //Serial.println("Error writing to flash");
    mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Error writing to flash",false);
    return p;
  } else {
    //Serial.println("Unknown error");
    mqttClient.publish(mqttTopicFingerprintEnrollStatus,"Unknown error",false);
    return p;
  }

  // wait 2s to avoid immediate toggle
  delay(2000);

  return true;
}

uint8_t getFingerprintID() {
  uint8_t p = finger.getImage();
  switch (p) {
    case FINGERPRINT_OK:
      //Serial.println("Image taken");
      mqttClient.publish(mqttTopicFingerprintDetectStatus,"Image taken",false);
      break;
    case FINGERPRINT_NOFINGER:
      //Serial.println("No finger detected");
      //mqttClient.publish(mqttTopicFingerprintDetectStatus,"No finger detected",false);
      // do nothing
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      // happens frequently due to SW Serial
      //Serial.println("Communication error");
      //mqttClient.publish(mqttTopicFingerprintDetectStatus,"Communication error",false);
      return p;
    case FINGERPRINT_IMAGEFAIL:
      //Serial.println("Imaging error");
      mqttClient.publish(mqttTopicFingerprintDetectStatus,"Imaging error",false);
      return p;
    default:
      //Serial.println("Unknown error");
      mqttClient.publish(mqttTopicFingerprintDetectStatus,"Unknown error",false);
      return p;
  }

  // OK success!

  p = finger.image2Tz();
  switch (p) {
    case FINGERPRINT_OK:
      //Serial.println("Image converted");
      mqttClient.publish(mqttTopicFingerprintDetectStatus,"Image converted",false);
      break;
    case FINGERPRINT_IMAGEMESS:
      //Serial.println("Image too messy");
      mqttClient.publish(mqttTopicFingerprintDetectStatus,"Image too messy",false);
      return p;
    case FINGERPRINT_PACKETRECIEVEERR:
      //Serial.println("Communication error");
      mqttClient.publish(mqttTopicFingerprintDetectStatus,"Communicatino error",false);
      return p;
    case FINGERPRINT_FEATUREFAIL:
      //Serial.println("Could not find fingerprint features");
      mqttClient.publish(mqttTopicFingerprintDetectStatus,"Could not find fingerprint features",false);
      return p;
    case FINGERPRINT_INVALIDIMAGE:
      //Serial.println("Invalid image");
      mqttClient.publish(mqttTopicFingerprintDetectStatus,"Invalid image",false);
      return p;
    default:
      //Serial.println("Unknown error");
      mqttClient.publish(mqttTopicFingerprintDetectStatus,"Unknown error",false);
      return p;
  }

  // OK converted!
  p = finger.fingerSearch();
  if (p == FINGERPRINT_OK) {
    //Serial.println("Found a print match!");
    mqttClient.publish(mqttTopicFingerprintDetectStatus,"Found a match!",false);
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    //Serial.println("Communication error");
    mqttClient.publish(mqttTopicFingerprintDetectStatus,"Communication error",false);
    return p;
  } else if (p == FINGERPRINT_NOTFOUND) {
    //Serial.println("Did not find a match");
    mqttClient.publish(mqttTopicFingerprintDetectStatus,"Did not find a match",false);
    return p;
  } else {
    //Serial.println("Unknown error");
    mqttClient.publish(mqttTopicFingerprintDetectStatus,"Unknown error",false);
    return p;
  }

  // found a match!
  //Serial.print("Found ID #"); Serial.print(finger.fingerID);
  //Serial.print(" with confidence of "); Serial.println(finger.confidence);
  toggleRelay();

  // wait for 2s to avoid that fingerprint toggles again immediately
  delay(2000);

  return finger.fingerID;
}
