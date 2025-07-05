/**
 * @file main.cpp
 * @brief nRF24 test sender
 * @details This sketch connects to a WiFi network and an MQTT broker, subscribes to topics, and sends messages using the nRF24 module.
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include "WifiInfo.h"
#include "MqttInfo.h"

// speed of serial interface for debug messages
#define SERIAL_SPEED 74880

// nRF24 CE/CSN pins
const uint8_t PIN_CE  = 15; // D8 on D1 mini (SPI CS)
const uint8_t PIN_CSN = 0;  // D3 on D1 mini (GPIO0)

// global WiFi, MQTT and RF24 client objects
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);
RF24         radio(PIN_CE, PIN_CSN);  // create an RF24 object, CE, CSN

// global buffer object
char buffer[256];

// MQTT callback function
void mqttCallback(const char topic[], byte* payload, unsigned int length);

// nRF24 default address (5 bytes)
const byte nRF24Address[6] = "hageg";

void setup() {
  // put your setup code here, to run once:
  Serial.begin(SERIAL_SPEED);
  delay(100);
  Serial.println();
  Serial.println(F("nRF24 testsender started"));

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

  Serial.println("");
  Serial.print(F("Connected to WiFi, IP address="));
  Serial.println(WiFi.localIP());

  // connect to MQTT broker
  char fullMqttClientName[128];
  sprintf(fullMqttClientName,"%s-%d",MQTT_CLIENT_ID,ESP.getChipId());
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

  // subscribe to topics
  mqttClient.subscribe(topicSubscribeTransmitPayload);
  mqttClient.subscribe(topicSubscribeSetAddress);

  // start nRF24 radio
  radio.begin();
  radio.setPayloadSize(16);               // set payload size to 16 bytes

  radio.stopListening();                  // set module as transmitter
  radio.openWritingPipe(nRF24Address);    // set the default address
}

void loop() {
  mqttClient.loop();
  delay(100);  
}

// MQTT callback function
void mqttCallback(const char topic[], byte* payload, unsigned int length) {
  // ignore zero length payloads to avoid endless loop
  if(length==0) {
    Serial.println(F("MQTT callback: zero length payload - ignoring"));
    return;
  }

  char topicBuffer[strlen(topic)+1];
  strcpy(topicBuffer,topic);

  char payloadString[length+1];
  strncpy(payloadString,(char*)payload,length);
  payloadString[length] = 0;

  sprintf(buffer,"MQTT message arrived [%s] value=%s",topic,payloadString);
  Serial.println(buffer);

  if(strcmp(topic,topicSubscribeSetAddress)==0) {
    if(length>5) {
      Serial.println(F("MQTT callback: nRF24 address too long - ignoring"));
      return;
    }

    radio.openWritingPipe((const uint8_t*)payloadString); // set the new nRF24 address
  }

  if(strcmp(topic,topicSubscribeTransmitPayload)==0) {
    radio.write(payloadString, sizeof(payloadString));// send message
  }
}