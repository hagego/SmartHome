#include <EEPROM.h>

/*
  ESP8266 code to control a rabbit feeder based on a hatch
*/

// mosquitto 1.3.4 speaks MQTT Version 3.1
// change to MQTT_VERSION MQTT_VERSION_3_1_1 after upgrade to 1.3.5 or higher


#define MQTT_VERSION MQTT_VERSION_3_1

#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>
#include <DHT.h>

// include WLAN authentification information
#include "WifiInfo.h"

#ifndef WIFI_SSID
#define WIFI_SSID "my_ssid"
#define WIFI_PSK  "my_psk"
#endif

const char* SSID     = WIFI_SSID;
const char* password = WIFI_PSK;

// MQTT broker IP address
const char* mqttServer     = "192.168.178.27";
const char* mqttClientName = "rabbitfeeder";

// MQTT topics
const char* topicVbat          = "rabbitfeeder/vbat";
const char* topicTemperature   = "rabbitfeeder/temperature";
const char* topicHatchCmd      = "rabbitfeeder/hatchCmd";
const char* topicConnect       = "rabbitfeeder/connect";

// resistor values for battery voltage measurement
const double ADC_RESISTOR_EXTERNAL = 1E6;
const double ADC_RESISTOR_INTERNAL = 220E3;
const double ADC_RESISTOR_MEASURE  = 100E3;

const int ANGLE_OPEN   = 100;
const int ANGLE_CLOSED = 20;

// pin definitions (GPIO0-GPIO15 all have internal pull-ups)
const int pinServoCtrl   = 4;  // GPIO04 servo control, D2 on D1 mini
const int pinServoPower  = 14; // GPIO14 servo power, D5 on D1 mini
const int pinDHT22       = 13; // GPIO13 DHT22 data, D7 on D1 mini

// EEPROM addresses
const int eepromAddressTemperature = 0; // sinlge byte
const int eepromAddressDoorStatus  = 1; // sinlge byte

// EEPROM door status
const int eepromDoorStatusOpen   = 0;
const int eepromDoorStatusClosed = 1;

// deep sleep period
const unsigned long SLEEP_PERIOD = 3600000000; // 3600 seconds = 1h
const unsigned long FEED_PERIOD  = 2000000;    // 5 seconds

// temperature measure frequency
byte TEMPERATURE_MEASURE_FREQUENCY = 2;  // measure every 2nd wake-up

// global flag if mqtt callback was received
boolean mqttCallbackReceived = false;

// function definitions
void callback(char* topic, byte* payload, unsigned int length);
int  processCmd(String cmd);

WiFiClient espClient;
PubSubClient client(espClient);
  
void setup() {
  // setup serial
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println();
  Serial.println("RabbitFeeder started");

  // setup servo power control => OFF
  pinMode(pinServoPower, OUTPUT);
  digitalWrite(pinServoPower,LOW);
  
     
  // Connect to WiFi network
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to SID ");
  Serial.println(SSID);
  WiFi.begin(SSID, password);

  const int COUNTER_MAX = 20;
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED && counter<COUNTER_MAX) {
    delay(500);
    Serial.print(".");
    counter++;
  }

  if(counter>=COUNTER_MAX) {
    Serial.print("Connection failed - sleeping again");
    ESP.deepSleep(SLEEP_PERIOD,RF_CAL);
  }
  Serial.println("");
  Serial.print("Connected to WiFi, IP address=");
  Serial.println(WiFi.localIP());

  client.setServer(mqttServer, 1883);
  client.setCallback(callback);
  mqttCallbackReceived = false;

  char buffer[128];
  sprintf(buffer,"%s-%d",mqttClientName,ESP.getChipId());
  Serial.print("Attempting MQTT connection to broker at ");
  Serial.print(mqttServer);
  Serial.print(" as client ");
  Serial.println(buffer);
  // Attempt to connect
  if (client.connect(buffer)) {
    Serial.println("MQTT connected");
    // Once connected, publish an announcement...
    client.publish(topicConnect, "connected");

    // subscribe for commands for hatch control
    client.subscribe(topicHatchCmd);

    // check if door is open and must be closed again
    EEPROM.begin(2);
    byte value = EEPROM.read(eepromAddressDoorStatus);
    EEPROM.end();
    if(value == eepromDoorStatusOpen) {
      closeHatch();
    }

    // MQTT client processing loop
    int i;
    for(i=0 ; (i<20 && mqttCallbackReceived==false) ; i++) {
      client.loop();
      delay(100);      
    }
    Serial.print("finished MQTT loop after steps: ");
    Serial.println(i);

    // measure battery voltage and publish
    measureVbat();

    // read temperature on DHT11
    EEPROM.begin(1);
    value = EEPROM.read(eepromAddressTemperature);
    if(value >= TEMPERATURE_MEASURE_FREQUENCY) {
      Serial.println("triggering temperature measure");
      readTemperature();
      value = 0;
    }
    value++;
    EEPROM.write(eepromAddressTemperature,value);
    EEPROM.end();
      
    client.disconnect();
    WiFi.disconnect();
  }
  else {
    Serial.print("failed, state=");
    Serial.print(client.state());
  }

  Serial.print("now sleeping for ");
  Serial.print(SLEEP_PERIOD/1000000UL);
  Serial.println("s");
  // wake-up of deep sleep mode requires connection between GPIO16 (D0 on Mini 1)
  // and RST and is actually a reset of the chip
  pinMode(D0, WAKEUP_PULLUP);
  ESP.deepSleep(SLEEP_PERIOD,RF_CAL);
}

void loop() {
}


// MQTT subscription loopback
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("],payload: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  char payloadString[length+1];
  strncpy(payloadString,(char*)payload,length);
  payloadString[length] = 0;
    
  if(strcmp(topic,topicHatchCmd)==0) {
    if(strcmp(payloadString,"open")==0) {
      openHatch();
    }
  }

  mqttCallbackReceived = true;
}

// open the hatch door
void openHatch() {
  Serial.println("opening door");
  Servo servo;

  // make sure DHHT22 data pin is low during power enable
  // otherwise COM port at PC gets reset - GND shifts ?
  pinMode(pinDHT22, OUTPUT);
  digitalWrite(pinDHT22,LOW);
  delay(100);
  
  digitalWrite(pinServoPower,HIGH);
  servo.attach(pinServoCtrl);
  
  for(int angle=ANGLE_CLOSED ; angle<=ANGLE_OPEN ; angle += 10) {
    servo.write(angle);
    delay(100);
  }

  digitalWrite(pinServoPower,LOW);
  client.publish(topicHatchCmd, "opened",true);

  EEPROM.begin(2);
  EEPROM.write(eepromAddressDoorStatus,eepromDoorStatusOpen);
  EEPROM.end();
}

// close the hatch door
void closeHatch() {
  Serial.println("closing door");
  Servo servo;

  // make sure DHHT22 data pin is low during power enable
  // otherwise COM port at PC gets reset - GND shifts ?
  pinMode(pinDHT22, OUTPUT);
  digitalWrite(pinDHT22,LOW);
  delay(100);
    
  digitalWrite(pinServoPower,HIGH);
  servo.attach(pinServoCtrl);

  for(int angle=ANGLE_OPEN ; angle>=ANGLE_CLOSED ; angle -= 10) {
    servo.write(angle);
    delay(100);
  }  

  digitalWrite(pinServoPower,LOW);
  client.publish(topicHatchCmd, "closed",true);

  EEPROM.begin(2);
  EEPROM.write(eepromAddressDoorStatus,eepromDoorStatusClosed);
  EEPROM.end();
}

// read temperature on DHT11
void readTemperature()
{
    // make sure DHHT22 data pin is low during power enable
    // otherwise COM port at PC gets reset - GND shifts ?
    pinMode(pinDHT22, OUTPUT);
    digitalWrite(pinDHT22,LOW);
    delay(100);
  
    digitalWrite(pinServoPower,HIGH);
    delay(1000);
    
    DHT dht(pinDHT22,DHT22);
    dht.begin();
    
    float t = dht.readTemperature();

    digitalWrite(pinServoPower,LOW);
    
    Serial.print("temperature: ");
    Serial.println(t);

    // and publish to MQTT broker
    char buffer[10];
    sprintf(buffer,"%.1f",t);
    client.publish(topicTemperature, buffer);
    Serial.print("publishing to topic ");
    Serial.print(topicTemperature);
    Serial.print(": ");
    Serial.println(buffer);
}

// measure battery voltage and publish
void measureVbat() {
  int adcValue = analogRead(A0);
  Serial.print("ADC value: ");
  Serial.println(adcValue);
  double vbat = ((double)adcValue/1023.0)*(ADC_RESISTOR_MEASURE+ADC_RESISTOR_INTERNAL+ADC_RESISTOR_EXTERNAL)/ADC_RESISTOR_MEASURE;
  Serial.print("battery voltage [V]: ");
  Serial.println(vbat);
  char buffer[100];
  sprintf(buffer,"publishing to topic %s : %f",topicVbat,vbat);
  Serial.println(buffer);
  sprintf(buffer,"%f",vbat);
  client.publish(topicVbat, buffer);
}
