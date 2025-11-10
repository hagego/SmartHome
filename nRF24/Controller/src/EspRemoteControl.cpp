/**
 * @file EspReceiver.cpp
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <time.h>


#include "WifiInfo.h"
#include "MqttInfo.h"
#include "MessageBuffer.h"
#include "Debug.h"


// speed of serial interface for debug messages
#define SERIAL_SPEED 74880
 
// nRF24 CE/CSN pins
const uint8_t PIN_CE  = 15; // D8 on D1 mini (SPI CS)
const uint8_t PIN_CSN = 0;  // D3 on D1 mini (GPIO0)

// D1 mini pins with button connected
const uint8_t PIN_BUTTON1 = 5; // D1 on D1 mini (GPIO5)
const uint8_t PIN_BUTTON2 = 4; // D2 on D1 mini (GPIO4)


// nRF24 addresses to listen to
// 0:   <x>ctrl: transmit
// 1-5: <x>clnt: clients 1-5
uint8_t nRF24Addresses[][6] = {RF24_ADDR_SEND, RF24_ADDR_RECEIVE}; 

// nRF24 payload size
const uint8_t nRF24PayloadSize = 16; // max. 32 bytes possible

 // global WiFi, MQTT and RF24 client objects
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);
RF24         radio(PIN_CE, PIN_CSN);  // create an RF24 object, CE, CSN

// global MessageBuffer object
MessageBuffer messageBuffer;
 
 // global buffer object for sprinf and other string operations
 char buffer[256];

  // full MQTT client name (client ID + IP address)
 char mqttFullClientName[128];
 
 // MQTT callback function declaration
 void mqttCallback(const char topic[], byte* payload, unsigned int length);

 // register MQTT topics declaration
 void registerMqttTopics();

 // send nRF24 message function declaration
 void sendNRF24Message(uint8_t client_id, const char* message);

 // global flag if the controller is connected to WiFi network
 bool wifiConnected = false;

 
void setup() {
  // initialize serial interface
  Serial.begin(SERIAL_SPEED);
  delay(100);
  Serial.println();
  Serial.println(F("nRF24 controller started"));
 
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

  wifiConnected = (WiFi.status() == WL_CONNECTED);

  // if compiled in PORTABLE mode, continue even if no WiFi connection could be established
  #ifndef PORTABLE
    if(counter>=COUNTER_MAX) {
      // timeout reached. sleep and try again
      Serial.print(F("\nConnection failed and controller is not portable - rebooting..."));
      delay(5000);
      ESP.restart();
    }
  #endif
 
  if(!wifiConnected) {
    Serial.println(F("\nWIFI connection failed - continuing in offline mode"));
  }
  else {
    Serial.print(F("\nConnected to WiFi, IP address="));
    Serial.println(WiFi.localIP());
  
    // connect to MQTT broker
    sprintf(mqttFullClientName,"%s-%s",MQTT_CLIENT_ID,WiFi.localIP().toString().c_str());
    mqttClient.setServer(MQTT_SERVER,MQTT_PORT);
    mqttClient.setCallback(mqttCallback);

    sprintf(buffer,"Attempting MQTT connection to broker at %s as client %s",MQTT_SERVER,mqttFullClientName);
    Serial.println(buffer);

    // Attempt to connect
    if (!mqttClient.connect(mqttFullClientName)) {
      Serial.print(F("MQTT connect failed, rc="));
      Serial.print(mqttClient.state());

      ESP.restart();
    }

    Serial.println(F("MQTT connected"));
    // Once connected, publish an announcement...
    sprintf(buffer,"connected as %s",mqttFullClientName);
    mqttClient.publish(MqttInfo::topicPublishConnected, buffer);
    mqttClient.publish(MqttInfo::topicPublishIsAlive, mqttFullClientName);

    // subscribe to topics
    registerMqttTopics();
  }

  // setup local button pins for debug/development
  pinMode(PIN_BUTTON1, OUTPUT);
  digitalWrite(PIN_BUTTON1, LOW);
  pinMode(PIN_BUTTON2, INPUT_PULLUP);
  Serial.printf("status PIN2: %d\n", digitalRead(PIN_BUTTON2));

  // start nRF24 radio
  radio.begin();
 
  // listen to all addresses in nRF24Addresses
  for(uint8 i=0; i<sizeof(nRF24Addresses)/sizeof(nRF24Addresses[0]); i++) {
    Serial.printf("Listening to nRF24 address %d: %s\n",i,nRF24Addresses[i]);
    radio.openReadingPipe(i, nRF24Addresses[i]);
  }

  radio.setAutoAck(1);            // Ensure autoACK is enabled
  radio.setRetries(5,15);         // Max delay between retries & number of retries
  radio.setPALevel(RF24_PA_HIGH); // Set power level to high
  radio.setPayloadSize(nRF24PayloadSize);
  radio.startListening();         // set module as receiver

  Debug::enableMQTTDebug(true);
  Debug::log("MQTT debug messages enabled");

  Serial.println(F("Setup done"));
}
 
// Main loop

u32_t keepAliveCounter = 0;                // use simple counter to send keepalive messages to MQTT broker roughly every half hour
uint8_t ledStripState = 1;

void loop() {
  keepAliveCounter++;

  // try to reconnect to WIFI if we have initially managed to connect and we are now disconnected
  if (wifiConnected) {
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

          // resubscribe to topics
          registerMqttTopics();

          // Once connected, publish an announcement...
          mqttClient.publish(MqttInfo::topicPublishConnected, mqttFullClientName);
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
  }



  uint8_t pipe;
  while(radio.available(&pipe)) {
    char text[16] = {0};
    radio.read(&text, sizeof(text));
    // first byte is client ID
    uint8_t client_id = text[0];
    Debug::log("Payload received on address %d, client %d: %s",pipe,client_id,text+1);

    // MqttInfo::topicPublishClientMessage
    sprintf(buffer,"%s/%d/%c",MqttInfo::topicPublishClientMessage,client_id,text[1]);
    mqttClient.publish(buffer, text+3);
        
    if(text[1] == 'L' && text[3] == '1') {
      // client is ready to receive commands
      Debug::log("Client %d is ready to receive commands",client_id);

      MessageBuffer::Message message;
      if(messageBuffer.getMessage(client_id, &message)) {
        radio.stopListening();          // set module as transmitter
        radio.openWritingPipe(nRF24Addresses[0]); // Write to device address

        Debug::log("Sending message on address %s, client %d: %s",nRF24Addresses[0],client_id,message.content);

        char writeBuffer[nRF24PayloadSize] = {0};
        writeBuffer[0] = client_id;
        strncpy(writeBuffer+1, message.content, sizeof(writeBuffer)-2);
        if( radio.write( writeBuffer, sizeof(writeBuffer) ) ) {
          // delete message from buffer
          Debug::log("Message acknowledged by client %d, deleting from buffer",client_id);
          messageBuffer.deleteMessage(&message);
        }

        // switch to listen mode again
        for(uint8 i=0; i<sizeof(nRF24Addresses)/sizeof(nRF24Addresses[0]); i++) {
          radio.openReadingPipe(i, nRF24Addresses[i]);
        }

        radio.startListening();                   // set module as receiver again
      } // message was sent
    } // client is listening
  } // message was received

  // read local button state
  if(digitalRead(PIN_BUTTON2) == LOW) {
    // button pressed
    Debug::log("button pressed");
    sprintf(buffer,"L:%d",ledStripState);
    sendNRF24Message(255, buffer);
    ledStripState++;
    if(ledStripState>3) {
      ledStripState = 0;
    }
    delay(500); // debounce
  }
  
  if(wifiConnected) {
    // send keepalive message to MQTT broker and read local sensor roughly every 5 minutes
    if(keepAliveCounter >= 30000UL) {
      keepAliveCounter = 0;

      Serial.println(F("Sending ping"));
      mqttClient.publish(MqttInfo::topicPublishIsAlive, mqttFullClientName);
    }

    mqttClient.loop();
  }

  delay(10);
}

// register MQTT topics
void registerMqttTopics() {
  // subscribe to topics
  mqttClient.subscribe(MqttInfo::topicSubscribeClientCommand); // + is nRF24 client ID suffix
  mqttClient.subscribe(MqttInfo::topicSubscribeEnableMqttDebug);
}

// MQTT callback function
void mqttCallback(const char topic[], byte* payload, unsigned int length) {
  // ignore zero length payloads to avoid endless loop
  if(length==0) {
    Serial.println(F("MQTT callback: zero length payload - ignoring"));
    return;
  }

  // topic[] is a globally allocated buffer - copy to local buffer to avoid overwriting in next MQTT message
  char topicBuffer[strlen(topic)+1];
  strcpy(topicBuffer,topic);

  char payloadString[length+1];
  strncpy(payloadString,(char*)payload,length);
  payloadString[length] = 0;

  Debug::log("MQTT callback: message [%s] value=%s",topicBuffer,payloadString);

  // check for enable/disable MQTT debug messages topic (only if connected to WIFI)
  if(wifiConnected && strcmp(topic,MqttInfo::topicSubscribeEnableMqttDebug)==0) {
    if(payloadString[0]=='1') {
      Debug::enableMQTTDebug(true);
      Debug::log("MQTT debug messages enabled");
    }
    else if(payloadString[0]=='0') {
      Debug::enableMQTTDebug(false);
      Debug::log("MQTT debug messages disabled");
    }

    return;
  }

  // check for client command topic. A commnand is stored in the message buffer until it
  // could be successfully sent to the client with acknowledgement
  size_t prefixLen = strlen(MqttInfo::topicSubscribeClientCommand)-1; // exclude trailing #
  if(strncmp(topicBuffer, MqttInfo::topicSubscribeClientCommand, prefixLen)==0) {
    // extract client ID from topic
    uint8_t client_id = atoi(topicBuffer + prefixLen);
    Debug::log("MQTT command for client %d: %s",client_id,payloadString);

    // try to send message immediately if client is listening
    radio.stopListening();          // set module as transmitter
    radio.openWritingPipe(nRF24Addresses[0]); // Write to device address

    Debug::log("Sending message on address %s, client %d: %s",nRF24Addresses[0],client_id,payloadString);

    char writeBuffer[nRF24PayloadSize] = {0};
    writeBuffer[0] = client_id;
    strncpy(writeBuffer+1, payloadString, sizeof(writeBuffer)-2);
    radio.setRetries(4,2);         // Max delay between retries & number of retries
    if( radio.write( writeBuffer, sizeof(writeBuffer) ) ) {
      // command acknowledged by client - no need to store in buffer
      Debug::log("Message acknowledged by client %d, consider done",client_id);
    }
    else {
      // store message in buffer
      messageBuffer.addMessage(client_id, payloadString);
    }

    radio.startListening(); 

    return;
  }
}

void sendNRF24Message(uint8_t client_id, const char* message) {
  radio.stopListening();                    // set module as transmitter

  Debug::log("Sending message on address %s, client %d: %s",nRF24Addresses[0],client_id,message);

  char writeBuffer[nRF24PayloadSize] = {0};
  writeBuffer[0] = client_id;
  strncpy(writeBuffer+1, message, sizeof(writeBuffer)-2);

  if( client_id == 255 ) {
    // broadcast message - no ACK expected
    radio.write( writeBuffer, sizeof(writeBuffer),true );
    Debug::log("Broadcast message sent");
  }
  else {
    if( radio.write( writeBuffer, sizeof(writeBuffer) ) ) {
      Debug::log("Message acknowledged by client %d",client_id);

    }
    else {
      Debug::log("Message NOT acknowledged by client %d",client_id);
      // store message in buffer
      messageBuffer.addMessage(client_id, message);
    }
  }


  radio.startListening();                   // set module as receiver again
}