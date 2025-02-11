#include <Arduino.h>
#include <PubSubClient.h>
#include "ESP8266WiFi.h"
#include <WiFiClient.h>


// include WLAN authentification information/home/hagen/Maker/repos/SmartHome/RabbitHatchDoor/RabbitHatchDoor/WiFiInfo.h
#include "WifiInfo.h"

#ifndef WIFI_SSID
#define WIFI_SSID "my_ssid"
#define WIFI_PSK  "my_psk"
#endif

// speed of serial interface for debug messages
#define SERIAL_SPEED 74880

// MQTT client name
const char* mqttClientName = "MotionSensorWohnzimmer";

// MQTT topics
#define MQTT_PREFIX "motionsensor/wohnzimmer/"

// topics to publish
const char* mqttTopicPublishConnected          = MQTT_PREFIX "connected";                   // published after connect
const char* mqttTopicPublishAlive              = MQTT_PREFIX "alive";                       // published periodically as alive signal
const char* mqttTopicPublishMotionDeteced      = MQTT_PREFIX "motionDetected";              // published after motion sensor status changed

// gpio pin to read sensor level
#define SENSOR_PIN 5 // D1 mini D1

// MQTT broker IP address
const char* mqtt_server = "192.168.178.27";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// buffer for sprintfs
char buffer[512];

// remember old/previously read sensor level
int oldMotion = 0;


void setup() {
  // setup serial
  Serial.begin(SERIAL_SPEED);
  delay(100);
  Serial.println();
  Serial.println(F("Motion Sensor started"));

    // Connect to WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.hostname("MotionSensor");
  Serial.print(F("Connecting to SID "));
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PSK);

  static int COUNTER_MAX = 100;
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED && counter<COUNTER_MAX) {
    delay(100);
    Serial.print(".");
    counter++;
  }

  if(counter>=COUNTER_MAX) {
    Serial.println(F("Connection failed - restarting"));
    ESP.restart();
  }

  Serial.println("");
  Serial.print(F("Connected to WiFi, IP address="));
  Serial.println(WiFi.localIP());

    // connect to MQTT broker
  mqttClient.setServer(mqtt_server, 1883);
  
  char mqttFullClientName[64];
  sprintf(mqttFullClientName,"%s-%d",mqttClientName,(int)ESP.getChipId);
  sprintf(buffer,"Attempting MQTT connection to broker at %s as client %s",mqtt_server,mqttFullClientName);
  Serial.println(buffer);
    
  // Attempt to connect
  if (mqttClient.connect(mqttFullClientName)) {
    sprintf(buffer,"connected to MQTT broker at %s as client %s, local IP=%s",mqtt_server,mqttFullClientName,WiFi.localIP().toString().c_str());
    Serial.println(buffer);

    // Once connected, publish an announcement...
    mqttClient.publish(mqttTopicPublishConnected, buffer);
  }
  else {
    Serial.println(F("connection failed - restarting"));
    ESP.restart();
  }

  // configure sensor pin as input
  pinMode(SENSOR_PIN, INPUT);

  Serial.println(F("setup finished"));
}


void loop() {
  //Serial.println(F("next loop iteration"));

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

      sprintf(buffer,"%s-%d",mqttClientName,ESP.getChipId());
      if (mqttClient.connect(buffer)) {
        sprintf(buffer,"reconnected in loop() to MQTT broker at %s as client %s-%d, local IP=%s",mqtt_server,mqttClientName,ESP.getChipId(),WiFi.localIP().toString().c_str());
        Serial.println(buffer);

        // Once connected, publish an announcement...
        mqttClient.publish(mqttTopicPublishConnected, buffer);
      }
    }
    else {
      Serial.println("WIFI reconnect failed. Rebooting...");
      // reboot
      ESP.restart();
    }
  }

  if(!mqttClient.connected()) {
    Serial.println(F("MQTT connection lost - restarting"));
    ESP.restart();
  }

  // read out sensor level
  int motion = digitalRead(SENSOR_PIN);
  //sprintf(buffer,"motion signal: %d",motion);
  //Serial.println(buffer);

  // publish motion signal status if changed
  if (motion != oldMotion) {
    oldMotion = motion;
    sprintf(buffer,"motion signal status changed: %d",motion);
    Serial.println(buffer);
    motion==1 ? mqttClient.publish(mqttTopicPublishMotionDeteced, "on") : mqttClient.publish(mqttTopicPublishMotionDeteced, "off");
    delay(1000);
  }

  mqttClient.loop();
  delay(500);
}
