#include <Arduino.h>

#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
#include <ESPAsyncHTTPUpdateServer.h>

#include <PubSubClient.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PN532.h>

#include "Apdu.h"
#include "WifiInfo.h"
#include "MqttInfo.h"


// speed of serial interface for debug messages
#define SERIAL_SPEED 74880
 
// global objects
Adafruit_PN532           nfc(D4); // use HW SPI, pin D4 for CS
WiFiClient               wifiClient;
PubSubClient             mqttClient(wifiClient);
ESPAsyncHTTPUpdateServer updateServer;
AsyncWebServer           webServer(80);
char                     buffer[256];
char                     fullMqttClientName[128];
 
void toggleRelay() {
  Serial.println("Toggling relay");

  digitalWrite(D1, HIGH);
  delay(500);
  digitalWrite(D1, LOW);
  delay(500);
}


// MQTT callback function
void mqttCallback(const char topic[], byte* payload, unsigned int length) {
  char payloadString[length+1];
  strncpy(payloadString,(char*)payload,length);
  payloadString[length] = 0;

  sprintf(buffer,"MQTT message arrived [%s] value=%s",topic,payloadString);
  Serial.println(buffer);

  if(strcmp(topic,topicSubscribeCommand)==0) {
    if(strcmp(payloadString,"toggle")==0 || strcmp(payloadString,"ON")==0) {
      toggleRelay();
    }
    else {
      Serial.println(F(": unknown command - ignoring"));
    }
  }
  else {
    Serial.println(F(": unknown topic - ignoring"));
  }
}



// page not found handler for HTTP update server
void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

bool pnxBoardFound;
void setup() {
  // initialize serial interface
  Serial.begin(SERIAL_SPEED);
  delay(100);
  Serial.println();
  Serial.println(F("garagedoor server started"));

  SPI.setFrequency(1000000); // Set SPI frequency to 1 MHz
  pinMode(D1, OUTPUT);       // used for relay

  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    pnxBoardFound = false;
  }
  else {
    // Got ok data, print it out!
    Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX);
    Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC);
    Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);

    nfc.SAMConfig(); //configure board to read NFC stuff
    pnxBoardFound = true;
  }
  
 
  // Connect to WiFi network
  Serial.print(F("Connecting to SID "));
  Serial.println(WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSK);
 
  static int COUNTER_MAX = 100;
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED && counter<COUNTER_MAX) {
    delay(100);
    Serial.print(".");
    counter++;
  }

  if(counter>=COUNTER_MAX) {
    // timeout reached. sleep and try again
    Serial.print(F("Connection failed"));
    ESP.restart();
  }
 
  Serial.print(F("\nConnected to WiFi, IP address="));
  Serial.println(WiFi.localIP());
 
  // connect to MQTT broker
  sprintf(fullMqttClientName,"%s-%s",MQTT_CLIENT_ID,WiFi.localIP().toString().c_str());
  mqttClient.setServer(MQTT_SERVER,MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  sprintf(buffer,"Attempting MQTT connection to broker at %s as client %s",MQTT_SERVER,fullMqttClientName);
  Serial.println(buffer);

  // Attempt to connect
  if (!mqttClient.connect(fullMqttClientName)) {
    Serial.print(F("MQTT connect failed, rc="));
    Serial.print(mqttClient.state());

    ESP.restart();
  }

  Serial.println(F("MQTT connected"));
  // Once connected, publish an announcement...
  sprintf(buffer,"connected as %s",fullMqttClientName);
  mqttClient.publish(topicPublishConnected, buffer);
  mqttClient.publish(topicPublishIsAlive, fullMqttClientName);

  // setup the HTTP update server
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "garage door server ready for updates. Use /update to upload new firmware.");
  });


  webServer.onNotFound(notFound);

  //setup the updateServer with credentials
  updateServer.setup(&webServer);

  webServer.begin();
}
 
// Main loop
u32_t keepAliveCounter = 0;                // use simple counter to send keepalive messages to MQTT broker roughly every half hour
u_int8_t oldLocalMotionSensorState = 0;    // used to detect changes in local motion sensor state

unsigned long lastPingMillis = 0;
void loop() {
  keepAliveCounter++;

  // reconnect to WIFI if needed
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("WIFI disconnected");
    WiFi.begin(WIFI_SSID, WIFI_PSK);
    uint8_t timeout = 60;
    while (timeout && (WiFi.status() != WL_CONNECTED)) {
      timeout--;
      delay(1000);
    }
    if(WiFi.status() == WL_CONNECTED) {
      Serial.println("WIFI reconnected");

      if (mqttClient.connect(fullMqttClientName)) {
        sprintf(buffer,"reconnected in loop() as client %s",fullMqttClientName);
        Serial.println(buffer);

        // Once connected, publish an announcement...
        mqttClient.publish(topicPublishConnected, buffer);
      }
    }
    else {
      Serial.println("WIFI reconnect failed. Rebooting...");
      // reboot
      ESP.restart();
    }
  }

  if(!mqttClient.connected()) {
    Serial.println(F("MQTT connection lost - reconnecting"));

    // connect to MQTT broker
    mqttClient.setServer(MQTT_SERVER,MQTT_PORT);
    mqttClient.setCallback(mqttCallback);

    sprintf(buffer,"Attempting MQTT connection to broker at %s as client %s",MQTT_SERVER,fullMqttClientName);
    Serial.println(buffer);

    // Attempt to reconnect
    int counter=20;
    while (!mqttClient.connect(fullMqttClientName) && counter > 0) {
      sprintf(buffer,"MQTT connect failed. rc=%d  WiFi status=%d",mqttClient.state(),WiFi.status());
      Serial.println(buffer);

      counter--;
      delay(1000);
    }

    if (counter == 0) {
      Serial.println(F("MQTT connect failed, giving up"));
      delay(500);
      ESP.restart();
    }

    Serial.println(F("MQTT reconnected"));
    mqttClient.publish(topicPublishIsAlive, fullMqttClientName);

    // subscribe to topics
    mqttClient.subscribe(topicSubscribeCommand);
  }

    // Wait for an NFC card to be present
  if (pnxBoardFound && nfc.inListPassiveTarget()) {
    Serial.println("Found an NFC card!");

    uint8_t response[255];                   // Buffer to store the returned data
    uint8_t responseLength=sizeof(response); // Length of the response buffer  
    char buffer[255];                        // sprintf buffer

    // send APDU to NFC card (App)
    int rc = nfc.inDataExchange(apduGarageDoorOpener, sizeof(apduGarageDoorOpener), response, &responseLength);
    
    //printing out what was returnd by the card
    sprintf(buffer,"apduGarageDoorOpener success=%d length: %d.\nData:", rc, responseLength);
    Serial.println(buffer);
    nfc.PrintHexChar(response, responseLength);
    Serial.println();

    if(rc == 1) {
      Serial.println("APDU command sent successfully.");

      // Handle successful response
      if(responseLength == strlen(expectedResponse) && strncmp((char*)response, expectedResponse, responseLength) == 0) {
        Serial.println("Valid response received.");

        toggleRelay();
      }
    }
  }

  // send ping every 5 minutes
  unsigned long currentMillis = millis();
  if(currentMillis<lastPingMillis || currentMillis-lastPingMillis >= 300000UL) {
    lastPingMillis = currentMillis;

    mqttClient.publish(topicPublishIsAlive, fullMqttClientName);
    sprintf(buffer,"%d",WiFi.RSSI());
    mqttClient.publish(topicPublishRssi, buffer);
  }

  mqttClient.loop();
  delay(500);
}
