

/*
  ESP8266 code to measure CO concentration and communicate via MQTT
  */

// mosquitto 1.3.4 speaks MQTT Version 3.1
// change to MQTT_VERSION MQTT_VERSION_3_1_1 after upgrade to 1.3.5 or higher

#define MQTT_VERSION MQTT_VERSION_3_1

#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <SPI.h>

// include WLAN authetication information
#include "WifiInfo.h"

#ifndef WIFI_SSID
#define WIFI_SSID "my_ssid"
#define WIFI_PSK  "my_psk"
#endif

const char* ssid     = WIFI_SSID;
const char* password = WIFI_PSK;

// resistor values for battery voltage measurement
const double ADC_RESISTOR_EXTERNAL = 220E3;  // external between sensor output and A0
const double ADC_RESISTOR_INTERNAL = 220E3;  // internal in ESP8266 module btw. A0 and ADC input
const double ADC_RESISTOR_MEASURE  = 100E3;  // internal in ESP8266 module btw. ADC and GND


// OTA host name
const char* otaHostName = "COSensorHeatingRoom";

// MQTT broker IP address and client name
const char* mqtt_server    = "192.168.178.27";
const char* mqttClientName = "COSensorHeatingRoom";

// MQTT topics
const char* mqttTopicStatus  = "COSensor/HeatingRoom/status";
const char* mqttTopicVsensor = "COSensor/HeatingRoom/vsensor";

// MQTT callback function for subscribed topics
void mqttCallback(char* topic, byte* payload, unsigned int length);

// MQTT ping interval (ms)
const long mqttPingInterval = 60000;  // 1 minute

// measure interval (ms)
const long measureInterval  = 60000;  // 1 minute


WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

void sendPing();
void measureVbat();

void setup() {
  Serial.begin(115200);
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
  ArduinoOTA.setHostname(otaHostName);

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
      Serial.println("connected");
      mqttClient.publish(mqttTopicStatus, "started",true);
  }
  else {
    Serial.print("MQTT connection failed: ");
    Serial.println(mqttClient.state());
  }


}

void loop() {
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
    
  ArduinoOTA.handle();
  mqttClient.loop();
  sendPing();
  measureVbat();
}

////////////////////////////////////////////////////////////////////////////////////////

void sendPing()
{
  static unsigned long previousMillis = 0;

  // check MQTT connection status and reconnect if needed
  if(!mqttClient.connected()){
    Serial.println("MQTT client disconnected");
    while(!mqttClient.connected()){
      mqttClient.connect(mqttClientName);
      uint8_t timeout = 8;
      while (timeout && (!mqttClient.connected())){
        timeout--;
        delay(1000);
      }
      if(mqttClient.connected()){
        Serial.println("MQTT client reconnected");
        mqttClient.publish(mqttTopicStatus, "reconnected",true);
      }
    }
  }
  
  unsigned long currentMillis = millis();
 
  // send a ping if enough millis have elapsed
  if (currentMillis - previousMillis >= mqttPingInterval)
  {
    previousMillis = currentMillis;
    
    Serial.println("sending ping");
    mqttClient.publish(mqttTopicStatus, "ping",true);
  }
}

/**
 * callback for subscribed MQTT messages
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) { 
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// measure battery voltage and publish
void measureVbat() {
  static unsigned long previousMillis = 0;

  
  
  unsigned long currentMillis = millis();
 
  // send a ping if enough millis have elapsed
  if (currentMillis - previousMillis >= measureInterval)
  {
    previousMillis = currentMillis;
    
    Serial.println("measuring sensor voltage");
  
  int adcValue = analogRead(A0);
  Serial.print("ADC value: ");
  Serial.println(adcValue);
  double vbat = ((double)adcValue/1023.0)*(ADC_RESISTOR_MEASURE+ADC_RESISTOR_INTERNAL+ADC_RESISTOR_EXTERNAL)/ADC_RESISTOR_MEASURE;
  Serial.print("battery voltage [V]: ");
  Serial.println(vbat);
  char buffer[100];
  sprintf(buffer,"publishing to topic %s : %f",mqttTopicVsensor,vbat);
  Serial.println(buffer);
  sprintf(buffer,"%f",vbat);
  mqttClient.publish(mqttTopicVsensor, buffer);    
  }

}
