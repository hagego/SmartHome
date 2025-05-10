/**
 * @file main.cpp
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
 
 // nRF24 addresses (5 bytes)
 const byte nRF24Address[6] = "hageg";

 // nRF24 addresses to listen to
  // 0: hageg: generic
  // 1: 1moti: motion sensor 1
  // 2: 2moti: motion sensor 2
 const uint8_t nRF24Addresses[][6] = {"hageg", "1moti", "2moti"};
 
void setup() {
  // put your setup code here, to run once:
  Serial.begin(SERIAL_SPEED);
  delay(100);
  Serial.println();
  Serial.println(F("nRF24 receiver started"));
 
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
  mqttClient.subscribe(topicSubscribeSetAddress);

  // start nRF24 radio
  radio.begin();
 
  // listen to all addresses in nRF24Addresses
  for(uint8 i=0; i<sizeof(nRF24Addresses)/sizeof(nRF24Addresses[0]); i++) {
    Serial.printf("Listening to nRF24 address %d: %s\n",i,nRF24Addresses[i]);
    radio.openReadingPipe(i, nRF24Addresses[i]);
  }

  radio.setPayloadSize(16);
  radio.startListening();            // set module as receiver
 }
 
void loop() {
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

      sprintf(buffer,"%s-%d",MQTT_CLIENT_ID,ESP.getChipId());
      if (mqttClient.connect(buffer)) {
        sprintf(buffer,"reconnected in loop() to MQTT broker at %s as client %s-%d, local IP=%s",MQTT_SERVER,MQTT_CLIENT_ID,ESP.getChipId(),WiFi.localIP().toString().c_str());
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
    Serial.println(F("MQTT connection lost - restarting"));
    ESP.restart();
  }

  if(radio.available()) {
    char text[16] = {0};
    radio.read(&text, sizeof(text));
    Serial.print(F("nRF24 payload received: "));
    Serial.println(text);
    mqttClient.publish(topicPublishPayloadReceived, text);
  }

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

    radio.openReadingPipe(0,(const uint8_t*)payloadString); // set the new nRF24 address
  }
}

