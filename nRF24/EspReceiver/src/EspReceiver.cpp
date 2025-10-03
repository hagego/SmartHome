/**
 * @file EspReceiver.cpp
 */

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <bsec2.h>

#include "WifiInfo.h"
#include "MqttInfo.h"


// speed of serial interface for debug messages
#define SERIAL_SPEED 74880
 
 // nRF24 CE/CSN pins
const uint8_t PIN_CE  = 15; // D8 on D1 mini (SPI CS)
const uint8_t PIN_CSN = 0;  // D3 on D1 mini (GPIO0)

 // local motion sensor pin
const uint8_t PIN_MOTION_SENSOR = 16; // D0 on D1 mini (GPIO16)

// nRF24 addresses to listen to
// 0: hageg: generic
// 1: 1moti: remote sensor 1
// 2: 2moti: remote sensor 2
uint8_t nRF24Addresses[][6] = {"hageg", "1moti", "2moti"};

// nRF24 paload size
const uint8_t nRF24PayloadSize = 16; // max. 32 bytes possible

 // global WiFi, MQTT, RF24 and Bsecs client objects
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);
RF24         radio(PIN_CE, PIN_CSN);  // create an RF24 object, CE, CSN
Bsec2        envSensor;

 
 // global buffer object for sprinf and other string operations
 char buffer[256];
 
 // MQTT callback function
 void mqttCallback(const char topic[], byte* payload, unsigned int length);

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
 
 
void setup() {
  // initialize GPIO pins
  pinMode(PIN_MOTION_SENSOR, INPUT); // local motion sensor pin
  
  // initialize serial interface
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
 
  Serial.print(F("\nConnected to WiFi, IP address="));
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
  mqttClient.publish(topicPublishIsAlive, "ping");

  // subscribe to topics
  mqttClient.subscribe(topicSubscribeSetAddress);

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
  if (!envSensor.begin(BME68X_I2C_ADDR_HIGH, Wire))
  {
    checkBsecStatus(envSensor);
  }
	
	/*
	 *	The default offset provided has been determined by testing the sensor in LP and ULP mode on application board 3.0
	 *	Please update the offset value after testing this on your product 
	 */
	envSensor.setTemperatureOffset(TEMP_OFFSET_ULP);


  /* Subsribe to the desired BSEC2 outputs */
  if (!envSensor.updateSubscription(sensorList, ARRAY_LEN(sensorList), BSEC_SAMPLE_RATE_ULP))
  {
    checkBsecStatus(envSensor);
  }

  /* Whenever new data is available call the newDataCallback function */
  envSensor.attachCallback(newDataCallback);

  // start nRF24 radio
  radio.begin();
 
  // listen to all addresses in nRF24Addresses
  for(uint8 i=0; i<sizeof(nRF24Addresses)/sizeof(nRF24Addresses[0]); i++) {
    Serial.printf("Listening to nRF24 address %d: %s\n",i,nRF24Addresses[i]);
    radio.openReadingPipe(i, nRF24Addresses[i]);
  }

  radio.setPayloadSize(nRF24PayloadSize);
  radio.startListening();                   // set module as receiver

  // trigger first BSEC run
  if (!envSensor.run())
  {
    checkBsecStatus(envSensor);
  }
}
 
// Main loop

u32_t keepAliveCounter = 0;                // use simple counter to send keepalive messages to MQTT broker roughly every half hour
u_int8_t oldLocalMotionSensorState = 0;    // used to detect changes in local motion sensor state

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

  // check if there is data on the nRF24
  uint8_t pipeNum;
  while(radio.available(&pipeNum)) {
    char text[nRF24PayloadSize] = {0};
    radio.read(&text, sizeof(text));
    sprintf(buffer,"nRF24 data received on pipe %d: %s",pipeNum,text);
    Serial.println(buffer);

    if(pipeNum==1) {
      // data from remote sensor 1
      if(strncmp(text,"C",1)==0 ) {
        Serial.println(F("connected reported by remote sensor 1"));
        mqttClient.publish(topicPublishRemoteSensor1Connected, "connected");
      }

      if(strncmp(text,"M",1)==0) {
        Serial.println(F("Motion change detected by remote sensor 1"));
        if(strlen(text)>2) {
          if(text[2]=='0') {
            Serial.println(F("motion off detected by remote sensor 1"));
            mqttClient.publish(topicPublishRemoteSensor1MotionDetected, "off");
          }
          else {
            sprintf(buffer,"on#%c",text[2]);
            mqttClient.publish(topicPublishRemoteSensor1MotionDetected, buffer);
          }
        }
      }

      if(strncmp(text,"V",1)==0 && strlen(text)>2) {
        Serial.println(F("Battery voltage reported by remote sensor 1"));
        mqttClient.publish(topicPublishRemoteSensor1Voltage, text+2); // skip first two characters (V and colon)
      }
      
      if(strncmp(text,"I",1)==0 && strlen(text)>2) {
        Serial.println(F("illuminance reported by remote sensor 1"));
        mqttClient.publish(topicPublishRemoteSensor1Illuminance, text+2); // skip first two characters (I and colon)
      }

      // publish received text as debug information
      mqttClient.publish(topicPublishRemoteSensor1Debug, text);
    }
    else {
      mqttClient.publish(topicPublishPayloadReceived, text);  
    }
  }

    // read local motion sensor level
  int motion = digitalRead(PIN_MOTION_SENSOR);

  // publish motion signal status if changed
  if (motion != oldLocalMotionSensorState) {
    oldLocalMotionSensorState = motion;
    sprintf(buffer,"local motion sensor signal status changed: %d",motion);
    Serial.println(buffer);
    motion==1 ? mqttClient.publish(topicPublishSensorMotionDetected, "on") : mqttClient.publish(topicPublishSensorMotionDetected, "off");
  }

  // send keepalive message to MQTT broker and read local sensor roughly every 5 minutes
  if(keepAliveCounter >= 3000UL) {
    keepAliveCounter = 0;

    Serial.println(F("Sending ping"));
    sprintf(buffer,"%s-%d:%s",MQTT_CLIENT_ID,ESP.getChipId(),WiFi.localIP().toString().c_str());
    mqttClient.publish(topicPublishIsAlive, buffer);

    if (!envSensor.run())
    {
      Serial.println(F("BME .run() failed - checking status"));
      checkBsecStatus(envSensor);
    }
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


void newDataCallback(const bme68xData data, const bsecOutputs outputs, Bsec2 bsec)
{
  if (!outputs.nOutputs)
  {
    return;
  }

  Serial.println("BSEC outputs:\n\tTime stamp = " + String((int) (outputs.output[0].time_stamp / INT64_C(1000000))));
  for (uint8_t i = 0; i < outputs.nOutputs; i++)
  {
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
          mqttClient.publish(topicPublishSensorTemperature, buffer);
          break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
          sprintf(buffer,"%.1f", output.signal);
          Serial.print("\tCompensated humidity = ");Serial.println(buffer);
          mqttClient.publish(topicPublishSensorHumidity, buffer);
          break;
      case BSEC_OUTPUT_STATIC_IAQ:
          sprintf(buffer,"%.1f", output.signal);
          Serial.print("\tIAQ static = ");Serial.println(buffer);
          mqttClient.publish(topicPublishSensorIaq, buffer);

          sprintf(buffer,"%d", output.accuracy);
          Serial.print("\tIAQ static accuracy = ");Serial.println(buffer);
          mqttClient.publish(topicPublishSensorIaqAccuracy, buffer);
          break;
      case BSEC_OUTPUT_CO2_EQUIVALENT:
          sprintf(buffer,"%.1f", output.signal);
          Serial.print("\tCO2 Equivalent = ");Serial.println(buffer);
          mqttClient.publish(topicPublishSensorCo2, buffer);
          break;
      case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
          Serial.println("\tbVOC equivalent = " + String(output.signal));
          break;
      case BSEC_OUTPUT_COMPENSATED_GAS:
          sprintf(buffer,"%.1f", output.signal);
          Serial.print("\tCompensated gas = ");Serial.println(buffer);
          mqttClient.publish(topicPublishSensorGasResistance, buffer);
          break;
      default:
          break;
    }
  }
}

void checkBsecStatus(Bsec2 bsec)
{
  if (bsec.status < BSEC_OK)
  {
    Serial.println("BSEC error code : " + String(bsec.status));
  }
  else if (bsec.status > BSEC_OK)
  {
    Serial.println("BSEC warning code : " + String(bsec.status));
  }

  if (bsec.sensor.status < BME68X_OK)
  {
    Serial.println("BME68X error code : " + String(bsec.sensor.status));
  }
  else if (bsec.sensor.status > BME68X_OK)
  {
    Serial.println("BME68X warning code : " + String(bsec.sensor.status));
  }
}