/**
 * universal code for ESP8266 (D1 mini) to control a servo and measure temperature
 * and humidity with DHT22 sensor, controlled via MQTT. The D1 will try to connect to MQTT,
 * receive commands and publish sensor data and then go to sleep again
 */

#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <EEPROM.h>

// include WLAN authentification information
#include "WifiInfo.h"

// include MQTT parameters
#include "MqttInfo.h"

// sensor and battery measurements and servo control
#include "measure.h"
#include "servo.h"

// EEPROM addresses
#include "EepromInfo.h"

// speed of serial interface for debug messages
#define SERIAL_SPEED 74880

// pin definitions
const uint8_t pinPower       = 14; // GPIO14 servo and DHT22 power, D5 on D1 mini
const uint8_t pinDHT22       = 13; // GPIO13 DHT22 data, D7 on D1 mini
const uint8_t pinServoCtrl   = 4;  // GPIO04 servo control, D2 on D1 mini
const uint8_t pinManualOpen  = 5;  // GPIO05 connects key to manually open, D1 on D1 mini
const uint8_t pinManualClose = 12; // GPIO12 connects key to manually close, D6 on D1 mini

// global WiFi and MQTT client objects
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);

// use a global buffer for sprintfs
char buffer[256];

// exit routine. Disconnects, disbales power and goes to sleep
void exit();

// MQTT callback function
void mqttCallback(const char topic[], byte* payload, unsigned int length);

void setup() {
  // first check local push buttons for manual control
  bool localControl = false;
  char localCmd[10];
  
  pinMode(pinManualOpen, INPUT_PULLUP);
  pinMode(pinManualClose, INPUT_PULLUP);
  delay(1)                                                                                                                                                                                                                                                                         ;
    
  if(digitalRead(pinManualOpen) == LOW){ 
    localControl = true;
    strcpy(localCmd,"open");
  }
  if(digitalRead(pinManualClose) == LOW){ 
    localControl = true;
    strcpy(localCmd,"close");
  }

  // setup serial
  Serial.begin(SERIAL_SPEED);
  delay(100);
  Serial.println();
  Serial.println(F("Universal servo sensor started"));

  // initialize EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // init pins
  pinMode(pinDHT22, INPUT_PULLUP);
  pinMode(pinPower, OUTPUT);
  pinMode(D0, WAKEUP_PULLUP);

  // enable power for servo and DHT22
  digitalWrite(pinPower,HIGH);

    // process local servo command if needed
  if(localControl) {
    Serial.print(F("push button pressed, cmd="));
    Serial.println(localCmd);
    processServoCmd(pinServoCtrl,localCmd);
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
    exit();
  }

  Serial.println("");
  Serial.print(F("Connected to WiFi, IP address="));
  Serial.println(WiFi.localIP());

  // connect to MQTT broker
  char fullMqttClientName[128];
  sprintf(fullMqttClientName,"%s-%s",MQTT_CLIENT_ID,WiFi.localIP().toString().c_str());
  mqttClient.setServer(MQTT_SERVER,MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  sprintf(buffer,"Attempting MQTT connection to broker at %s as client %s",MQTT_SERVER,fullMqttClientName);
  Serial.println(buffer);

   // Attempt to connect
   if (!mqttClient.connect(fullMqttClientName)) {
    Serial.print(F("MQTT connect failed, rc="));
    Serial.print(mqttClient.state());

    exit();
   }

  Serial.println(F("MQTT connected"));
  // Once connected, publish an announcement...
  mqttClient.publish(topicPublishConnected, fullMqttClientName);

  // subscribe to topics
  mqttClient.subscribe(topicSubscribeSleepTime);
  mqttClient.subscribe(topicSubscribeAngleOpen);
  mqttClient.subscribe(topicSubscribeAngleClose);
  mqttClient.subscribe(topicSubscribeServoCommand);

  //mqttClient.publish(topicPublishDebug, "start loop");
  mqttClient.loop();

  // process local servo command if needed
  if(localControl) {
    sprintf(buffer,"%sReceived",topicSubscribeServoCommand);
    mqttClient.publish(buffer, localCmd,true);
    mqttClient.publish(topicPublishDoorControl,"local",true);
  }

  //mqttClient.publish(topicPublishDebug, "start temp measurement");
  measureTemperature(pinDHT22,mqttClient);
  //mqttClient.publish(topicPublishDebug, "start battery measurement");
  measureBatteryVoltage(mqttClient);
  //mqttClient.publish(topicPublishDebug, "battery measurement done");

  // give enough time to receive MQTT publications
  Serial.println(F("MQTT subsription loop started"));
  for(int i=0 ; i<10 ; i++) {
    mqttClient.loop();
    delay(100);      
  }
  Serial.println(F("MQTT subsription loop done"));
   
  exit();
}

void loop() {
  // nothing to do here
}

// code found on https://github.com/esp8266/Arduino/issues/6318
// ESP.deepSleep() is not working reliable on all modules
uint32_t*RT= (uint32_t *)0x60000700;

void DeepSleepNK(uint64 t_us)
{
  RT[4] = 0;
  *RT = 0;
  RT[1]=100;
  RT[3] = 0x10010;
  RT[6] = 8;
  RT[17] = 4;
  RT[2] = 1<<20;
  ets_delay_us(10);
  RT[1]=t_us>>3;
  RT[3] = 0x640C8;
  RT[4]= 0;
  RT[6] = 0x18;
  RT[16] = 0x7F;
  RT[17] = 0x20;
  RT[39] = 0x11;
  RT[40] = 0x03;
  RT[2] |= 1<<20;
  __asm volatile ("waiti 0");
}


// exit routine. Disconnects, disbales power and goes to sleep
void exit() {
  // disconnect from MQTT broker
  if (mqttClient.connected()) {
    mqttClient.disconnect();
    Serial.println(F("MQTT disconnected"));
  }

  // disconnect from WiFi network
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
    Serial.println(F("WiFi disconnected"));
  }

  // go to sleep
  uint8_t sleepTimeMinutes = EEPROM.read(EEPROM_SLEEP_TIME_MINUTES);
  u_int64_t sleepTime = 60UL*(u_int64_t)sleepTimeMinutes*1000000UL;
  sprintf(buffer,"Going to sleep for %d minutes",sleepTimeMinutes);
  Serial.println(buffer);

  // commit EEPROM data
  EEPROM.commit();
  EEPROM.end();

  // disable power for servo and DHT22
  digitalWrite(pinPower,LOW);

  delay(500); // give enough time to flush

  DeepSleepNK(sleepTime);
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

  if(strcmp(topic,topicSubscribeSleepTime)==0) {
    // check for override of sleep period
    uint32_t sleepPeriodSeconds = (uint32_t)atoi(payloadString);
    uint8_t  sleepPeriodMinutes = (uint8_t)(sleepPeriodSeconds/60);
    sprintf(buffer,"recived sleepPeriod: %d seconds %d minutes",sleepPeriodSeconds,sleepPeriodMinutes);
    Serial.println(buffer);

    // store sleep period in EEPROM
    EEPROM.write(EEPROM_SLEEP_TIME_MINUTES,sleepPeriodMinutes);
  }

  if(strcmp(topic,topicSubscribeAngleOpen)==0) {
    // store angle for open position in EEPROM
    byte angle = (byte)atoi(payloadString);
    sprintf(buffer,"recived angleOpen: %d",angle);
    Serial.println(buffer);

    EEPROM.write(EEPROM_ANGLE_OPEN,angle); // EEPROM already optimizes write of identical values
  }

  if(strcmp(topic,topicSubscribeAngleClose)==0) {
    // store angle for close position in EEPROM
    byte angle = (byte)atoi(payloadString);
    sprintf(buffer,"recived angleClose: %d",angle);
    Serial.println(buffer);

    EEPROM.write(EEPROM_ANGLE_CLOSE,angle); // EEPROM already optimizes write of identical values
  }

  if(strcmp(topic,topicSubscribeServoCommand)==0) {
    sprintf(buffer,"received servo command: %s",payloadString);
    Serial.println(buffer);

    processServoCmd(pinServoCtrl,payloadString);
    mqttClient.publish(topicPublishDoorControl,"remote",true);
  }

  // send back confirmation about received message and clear retained message
  sprintf(buffer,"publishing topic %sReceived",topicBuffer);
  Serial.println(buffer);

  sprintf(buffer,"%sReceived",topicBuffer);
  mqttClient.publish(buffer, payloadString,true);
  mqttClient.publish(topicBuffer,"",true);
}