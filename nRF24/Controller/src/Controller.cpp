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

#ifdef BSEC
#include <bsec2.h>
#endif


#include "WifiInfo.h"
#include "MqttInfo.h"
#include "MessageBuffer.h"
#include "Debug.h"


// speed of serial interface for debug messages
#define SERIAL_SPEED 74880
 
// nRF24 CE/CSN pins
const uint8_t PIN_CE  = 15; // D8 on D1 mini (SPI CS)
const uint8_t PIN_CSN = 0;  // D3 on D1 mini (GPIO0)

// pins for locally connected stuff
const uint8_t PIN_BUTTON1 = 5;         // local button connected to I2C pins, D1 on D1 mini (GPIO5)
const uint8_t PIN_BUTTON2 = 4;         // local button connected to I2C pins, D2 on D1 mini (GPIO4)
const uint8_t PIN_MOTION_SENSOR = 16;  // local motion sensor pin, D0 on D1 mini (GPIO16)


// nRF24 addresses to listen to
// 0:   <x>ctrl: transmit
// 1-5: <x>clnt: clients 1-5
uint8_t nRF24Addresses[][6] = {"ctrl ", RF24_ADDR_RECEIVE}; 

// nRF24 payload size
const uint8_t nRF24PayloadSize = 16; // max. 32 bytes possible

 // global objects
WiFiClient   wifiClient;              // WiFi client object
PubSubClient mqttClient(wifiClient);  // MQTT client object
RF24         radio(PIN_CE, PIN_CSN);  // RF24 object, CE, CSN

#ifdef BSEC
Bsec2        envSensor;               // BSEC2 object
#endif

// global MessageBuffer object
MessageBuffer messageBuffer;
 
// global buffer object for sprinf and other string operations
char buffer[256];

 // full MQTT client name (client ID + IP address)
char mqttFullClientName[128];
char mqttTopicName[128];


// function prototypes
// MQTT callback function declaration
void mqttCallback(const char topic[], byte* payload, unsigned int length);

// register MQTT topics declaration
void registerMqttTopics();

// send nRF24 message function declaration
void sendNRF24Message(uint8_t client_id, const char* message);

// global flag if the controller is connected to WiFi network
bool wifiConnected = false;

#ifdef BSEC
 /**
 * @brief : This function checks the BSEC status, prints the respective error code. Halts in case of error
 * @param[in] bsec  : Bsec2 class object
 */
void checkBsecStatus(Bsec2 bsec);

/**
 * @brief : This function is called by the BSEC library when a new output is available
 * @param[in] input     : BME68X sensor data before processing
 * @param[in] outputs   : Processed BSEC BSEC output data
 * @param[in] bsec      : Instance of BSEC2 calling the callback
 */
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec);
#endif


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

    delay(100);
    mqttClient.loop();
    delay(100);
  }

  #ifdef BSEC
    /* Desired subscription list of BSEC2 outputs */
    bsecSensor sensorList[] = {
            BSEC_OUTPUT_STABILIZATION_STATUS,
            BSEC_OUTPUT_RUN_IN_STATUS,
            BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
            BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
            BSEC_OUTPUT_STATIC_IAQ,
            BSEC_OUTPUT_CO2_EQUIVALENT,
            BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
            BSEC_OUTPUT_COMPENSATED_GAS
    };

    /* Initialize the BME680 sensor library and interfaces */
    Wire.begin();
    if (!envSensor.begin(BME68X_I2C_ADDR_HIGH, Wire)) {
      Debug::log("BME68x.begin() failed - checking status");
      checkBsecStatus(envSensor);
    }
    else {
      Debug::log("BME68x sensor initialized");
    }
    
    /*
    *	The default offset provided has been determined by testing the sensor in LP and ULP mode on application board 3.0
    *	Please update the offset value after testing this on your product 
    */
    envSensor.setTemperatureOffset(TEMP_OFFSET_ULP);


    /* Subsribe to the desired BSEC2 outputs */
    if (!envSensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), BSEC_SAMPLE_RATE_ULP)) {
      checkBsecStatus(envSensor);
    }

    /* Whenever new data is available call the newDataCallback function */
    envSensor.attachCallback(newDataCallback);

      // trigger first BSEC run
    if (!envSensor.run()) {
      Debug::log("BSEC run() failed - checking status");
      checkBsecStatus(envSensor);
    }
    else {
      Debug::log("BSEC2 initialized and first run triggered");
    }
  #else
    // setup local button pins for debug/development
    pinMode(PIN_BUTTON1, OUTPUT);
    digitalWrite(PIN_BUTTON1, LOW);
    pinMode(PIN_BUTTON2, INPUT_PULLUP);
    Serial.printf("status PIN2: %d\n", digitalRead(PIN_BUTTON2));
  #endif

  // start nRF24 radio
  radio.begin();
 

  // listen to address 1 only
  radio.openReadingPipe(1, nRF24Addresses[1]);
  
  radio.setAutoAck(1);            // Ensure autoACK is enabled
  radio.enableAckPayload();       // Allow optional ack payloads
  radio.enableDynamicAck();      // Allow dynamic ACKs
  radio.setRetries(5,15);         // Max delay between retries & number of retries
  radio.setPALevel(RF24_PA_HIGH); // Set power level to high
  radio.setPayloadSize(nRF24PayloadSize);
  radio.startListening();         // set module as receiver

  Serial.println(F("Setup done"));
}
 
// Main loop

u32_t keepAliveCounter = 0;                // use simple counter to send keepalive messages to MQTT broker roughly every half hour
uint8_t ledStripState = 1;
u_int8_t oldLocalMotionSensorState = 0;    // used to detect changes in local motion sensor state

void loop() {
  keepAliveCounter++;

  // try to reconnect to WIFI if we have initially managed to connect and we are now disconnected
  #ifndef PORTABLE
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
  #endif


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
    
    if(   text[1] == 'C' && text[3] == '1'
       && client_id < MqttInfo::NUM_CLIENT_PREFIXES && strlen(MqttInfo::mqttPrefixForClientId[client_id])>0) {
      // client connected
      sprintf(mqttTopicName, "%s%s", MqttInfo::mqttPrefixForClientId[client_id], MqttInfo::topicPublishSensorConnected);
      mqttClient.publish(mqttTopicName, "connected");
    }

    if(   text[1] == 'V' 
       && client_id < MqttInfo::NUM_CLIENT_PREFIXES && strlen(MqttInfo::mqttPrefixForClientId[client_id])>0) {
      sprintf(mqttTopicName, "%s%s", MqttInfo::mqttPrefixForClientId[client_id], MqttInfo::topicPublishSensorBattery);

      // some clients report battery voltage in mV, convert to V
      float voltage = atof(text+3);
      if(voltage>10) {
        // assume mV
        voltage = voltage / 1000.0;
      }
      sprintf(buffer,"%.1f",voltage);
      mqttClient.publish(mqttTopicName, buffer);
    }

    if(   text[1] == 'I' 
       && client_id < MqttInfo::NUM_CLIENT_PREFIXES && strlen(MqttInfo::mqttPrefixForClientId[client_id])>0) {
      sprintf(mqttTopicName, "%s%s", MqttInfo::mqttPrefixForClientId[client_id], MqttInfo::topicPublishSensorIlluminance);
      mqttClient.publish(mqttTopicName, text+3);
    }

    if(   text[1] == 'D' 
       && client_id < MqttInfo::NUM_CLIENT_PREFIXES && strlen(MqttInfo::mqttPrefixForClientId[client_id])>0) {
      sprintf(mqttTopicName, "%s%s", MqttInfo::mqttPrefixForClientId[client_id], MqttInfo::topicPublishSensorTemperature);
      mqttClient.publish(mqttTopicName, text+3);
    }

    if(   text[1] == 'M' && text[3] != '0'
       && client_id < MqttInfo::NUM_CLIENT_PREFIXES && strlen(MqttInfo::mqttPrefixForClientId[client_id])>0) {
      // client connected
      sprintf(mqttTopicName, "%s%s", MqttInfo::mqttPrefixForClientId[client_id], MqttInfo::topicPublishSensorMotionDetected);
      sprintf(buffer,"on#%c",text[3]);
      mqttClient.publish(mqttTopicName, buffer);
    }

    if(text[1] == 'L' && text[3] == '1') {
      // client is ready to receive commands
      Debug::log("Client %d is ready to receive commands",client_id);

      MessageBuffer::Message message;
      if(client_id!=0 && messageBuffer.getMessage(client_id, &message)) {
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

        // listen to address 1 only
        radio.openReadingPipe(1, nRF24Addresses[1]);

        radio.startListening();                   // set module as receiver again
      } // message was sent
    } // client is listening
  
  } // message was received


  // read local motion sensor level
  #ifdef LOCAL_MOTION_SENSOR
  int motion = digitalRead(PIN_MOTION_SENSOR);

  // publish motion signal status if changed
  if (motion != oldLocalMotionSensorState) {
    oldLocalMotionSensorState = motion;
    uint8_t client_id = 0;
    char mqttTopicName[128];
    sprintf(mqttTopicName, "%s%s", MqttInfo::mqttPrefixForClientId[client_id], MqttInfo::topicPublishSensorMotionDetected);
    Debug::log("local motion sensor signal status changed: %d",motion);
    motion==1 ? mqttClient.publish(mqttTopicName, "on") : mqttClient.publish(mqttTopicName, "off");
  }
  #endif


  // read local button state
  #ifndef BSEC
  if(digitalRead(PIN_BUTTON2) == LOW) {
    // button pressed
    Debug::log("local button pressed");
    sprintf(buffer,"L:%d",ledStripState);
    sendNRF24Message(255, buffer);
    ledStripState++;
    if(ledStripState>3) {
      ledStripState = 0;
    }
    delay(500); // debounce
  }
  #endif
  
  if(wifiConnected) {
    // send keepalive message to MQTT broker and read local sensor roughly every 5 minutes
    if(keepAliveCounter >= 3000UL) {
      keepAliveCounter = 0;
      Debug::log("5min interval reached");

      #ifdef BSEC
      if (!envSensor.run()) {
        Debug::log("BME .run() failed - checking status");
        checkBsecStatus(envSensor);
      }
      #endif

      mqttClient.publish(MqttInfo::topicPublishIsAlive, mqttFullClientName);
    }
    mqttClient.loop();
  }

  delay(100);
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

    sendNRF24Message(client_id, payloadString);
    return;
  }
}

void sendNRF24Message(uint8_t client_id, const char* message) {
  radio.stopListening();                    // set module as transmitter
  radio.openWritingPipe(nRF24Addresses[0]); // Write to device address

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
            uint8_t pipe;
      if(radio.available(&pipe)) {
        // check for ack payload
        char ackPayload[nRF24PayloadSize] = {0};
        radio.read(ackPayload, sizeof(ackPayload));
        Debug::log("Message acknowledged by client %d thru ack payload",ackPayload[0]);
      }
      else {
        Debug::log("Message acknowledged by client %d, but no ack payload received",client_id);
      }
    }
    else {
      Debug::log("Message NOT acknowledged by client %d",client_id);
      // store message in buffer
      messageBuffer.addMessage(client_id, message);
    }
  }


  radio.startListening();                   // set module as receiver again
}

#ifdef BSEC
void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
  if (!outputs.nOutputs) {
    return;
  }

  // MQTT prefix for local sensor is mapped to RF24 client ID 0
  char mqttTopicName[128];

  Serial.println("BSEC outputs:\n\tTime stamp = " + String((int) (outputs.output[0].time_stamp / INT64_C(1000000))));
  for (uint8_t i = 0; i < outputs.nOutputs; i++) {
    const bsecData output  = outputs.output[i];
    switch (output.sensor_id)
    {
      case BSEC_OUTPUT_IAQ:
          
          break;
      case BSEC_OUTPUT_STABILIZATION_STATUS:
          Serial.println("\tStabilization status = " + String(output.signal));
          break;
      case BSEC_OUTPUT_RUN_IN_STATUS:
          Serial.println("\tRun in status = " + String(output.signal));
          break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
          sprintf(buffer,"%.1f", output.signal);
          Serial.print("\tCompensated temperature = ");Serial.println(buffer);
          sprintf(mqttTopicName, "%s%s", MqttInfo::mqttPrefixForClientId[0], MqttInfo::topicPublishSensorTemperature);
          mqttClient.publish(mqttTopicName, buffer);
          break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
          sprintf(buffer,"%.1f", output.signal);
          Serial.print("\tCompensated humidity = ");Serial.println(buffer);
          sprintf(mqttTopicName, "%s%s", MqttInfo::mqttPrefixForClientId[0], MqttInfo::topicPublishSensorHumidity);
          mqttClient.publish(mqttTopicName, buffer);
          break;
      case BSEC_OUTPUT_STATIC_IAQ:
          sprintf(buffer,"%.1f", output.signal);
          Serial.print("\tIAQ static = ");Serial.println(buffer);
          sprintf(mqttTopicName, "%s%s", MqttInfo::mqttPrefixForClientId[0], MqttInfo::topicPublishSensorIaq);
          mqttClient.publish(mqttTopicName, buffer);

          sprintf(buffer,"%d", output.accuracy);
          Serial.print("\tIAQ static accuracy = ");Serial.println(buffer);
          sprintf(mqttTopicName, "%s%s", MqttInfo::mqttPrefixForClientId[0], MqttInfo::topicPublishSensorIaqAccuracy);
          mqttClient.publish(mqttTopicName, buffer);
          break;
      case BSEC_OUTPUT_CO2_EQUIVALENT:
          sprintf(buffer,"%.1f", output.signal);
          Serial.print("\tCO2 Equivalent = ");Serial.println(buffer);
          sprintf(mqttTopicName, "%s%s", MqttInfo::mqttPrefixForClientId[0], MqttInfo::topicPublishSensorCo2);
          mqttClient.publish(mqttTopicName, buffer);
          break;
      case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
          Serial.println("\tbVOC equivalent = " + String(output.signal));
          break;
      case BSEC_OUTPUT_COMPENSATED_GAS:
          sprintf(buffer,"%.1f", output.signal);
          Serial.print("\tCompensated gas = ");Serial.println(buffer);
          sprintf(mqttTopicName, "%s%s", MqttInfo::mqttPrefixForClientId[0], MqttInfo::topicPublishSensorGasResistance);
          mqttClient.publish(mqttTopicName, buffer);
          break;
      default:
          break;
    }
  }
}

void checkBsecStatus(Bsec2 bsec)
{
  if (bsec.status < BSEC_OK) {
    Serial.println("BSEC error code : " + String(bsec.status));
  }
  else if (bsec.status > BSEC_OK) {
    Serial.println("BSEC warning code : " + String(bsec.status));
  }

  if (bsec.sensor.status < BME68X_OK) {
    Serial.println("BME68X error code : " + String(bsec.sensor.status));
  }
  else if (bsec.sensor.status > BME68X_OK) {
    Serial.println("BME68X warning code : " + String(bsec.sensor.status));
  }
}
#endif