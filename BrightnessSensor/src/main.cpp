/*
  ESP8266 code to measure brightness using a BH1750 sensor and publish the value to a MQTT broker.
*/

#define MQTT_PREFIX "brightnesssensor/terrace/"

// MQTT client name
const char* mqtt_client = "brightnesssensor";

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <BH1750.h>
#include <Wire.h>


// include WLAN authentification information/home/hagen/Maker/repos/SmartHome/RabbitHatchDoor/RabbitHatchDoor/WiFiInfo.h
#include "WifiInfo.h"

#ifndef WIFI_SSID
#define WIFI_SSID "my_ssid"
#define WIFI_PSK  "my_psk"
#endif

// speed of serial interface for debug messages
#define SERIAL_SPEED 74880

// MQTT broker IP address
const char* mqtt_server = "192.168.178.27";

// MQTT topics
// publish
const char* topicConnected        = MQTT_PREFIX "connected";
const char* topicBrightness       = MQTT_PREFIX "brightness";

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

char buffer[256];

// measure brightness using BH1750 sensor
void readBrightness() {
  Serial.println(F("readBrightness()"));
  delay(10); // ensure enough time for Serial output in case of error

  BH1750 lightMeter(0x23); // Address 0x23 is the default address of the sensor
  Serial.println(F("lighMeter created"));
  delay(10); // ensure enough time for Serial output in case of error

  if (lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE)) {
    Serial.println(F("BH1750 Advanced begin"));
    delay(1000); // ensure enough time for Serial output in case of error and measurement
    if (lightMeter.measurementReady(true)) {
      float lux = lightMeter.readLightLevel();
      sprintf(buffer,"publishing to topic %s : %.1f",topicBrightness,lux);
      Serial.println(buffer);
      sprintf(buffer,"%.1f",lux);
      mqttClient.publish(topicBrightness, buffer);
    } else {
      Serial.println(F("Error reading BH1750 light level"));
    }
  } else {
    Serial.println(F("Error initialising BH1750"));
  }
  delay(10); // ensure enough time for Serial output in case of error
}


  
void setup() {
  // setup serial
  Serial.begin(SERIAL_SPEED);
  delay(100);
  Serial.println();
  Serial.println();
  Serial.println(F("brightness sensor started"));
  
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
    Serial.print(F("Connection failed - rebooting"));
    ESP.restart();
  }

  Serial.println("");
  Serial.print(F("Connected to WiFi, IP address="));
  Serial.println(WiFi.localIP());

  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setKeepAlive(300);   // set MQTT keep alive to 5 minutes

  char mqttFullClientName[128];
  sprintf(mqttFullClientName,"%s-%d",mqtt_client,ESP.getChipId());

  sprintf(buffer,"attempting to connect to MQTT broker at %s as client %s",mqtt_server,mqttFullClientName);
  Serial.println(buffer);
    
  // Attempt to connect
  if (mqttClient.connect(mqttFullClientName)) {
    // Once connected, publish an announcement...
    sprintf(buffer,"connected to MQTT broker at %s as client %s, local IP=%s",mqtt_server,mqttFullClientName,WiFi.localIP().toString().c_str());
    Serial.println(buffer);
    mqttClient.publish(topicConnected, WiFi.localIP().toString().c_str());
  }
  else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
  }

  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin(); // uses default pins SDA=GPIO4=D2, SCL=GPIO5=D1

  Serial.println(F("Wire.begin() done"));
  delay(10); // ensure enough time for Serial output in case of error
}

int counter = 15;
void loop() {
  Serial.println(F("loop iteration starting"));
  counter++;

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

      sprintf(buffer,"%s-%d",mqtt_client,ESP.getChipId());
      if (mqttClient.connect(buffer)) {
        sprintf(buffer,"reconnected in loop() to MQTT broker at %s as client %s-%d, local IP=%s",mqtt_server,mqtt_client,ESP.getChipId(),WiFi.localIP().toString().c_str());
        Serial.println(buffer);

        // Once connected, publish an announcement...
        mqttClient.publish(topicConnected, WiFi.localIP().toString().c_str());
      }
    }
    else {
      Serial.println("WIFI reconnect failed. Rebooting...");
      // reboot
      ESP.restart();
    }
  }
  Serial.println(F("WIFI connected"));

  if(!mqttClient.connected()) {
    Serial.println(F("MQTT connection lost - restarting"));
    ESP.restart();
  }
  mqttClient.loop();

  if(counter>15) {
    counter = 0;
    // read brightness
    readBrightness();
  }

  Serial.println(F("sleeping for 1min"));
  delay(60000); // 1min
}


