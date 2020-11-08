/*
  ESP8266 code to control a door of a rabbit hatch based on periodically
  querying an openhab server for commands
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
const char* mqtt_server = "192.168.178.27";

// MQTT client name
const char* mqtt_client = "rabbithutch";

// MQTT topics
const char* topicTemperature = "rabbithutch/temperature";
const char* topicVbat        = "rabbithutch/vbat";
const char* topicDoor        = "rabbithutch/door";
const char* topicAngle       = "rabbithutch/angle";
const char* topicAngleOpen   = "rabbithutch/angleOpen";
const char* topicAngleClose  = "rabbithutch/angleClose";

// EEPROM addresses
const int EEPROM_ANGLE_LAST          = 0;
const int EEPROM_ANGLE_OPEN          = 1;
const int EEPROM_ANGLE_CLOSE         = 2;
const int EEPROM_TEMPERATURE_COUNTER = 3;

// resistor values for battery voltage measurement
const double ADC_RESISTOR_EXTERNAL = 1E6;
const double ADC_RESISTOR_INTERNAL = 220E3;
const double ADC_RESISTOR_MEASURE  = 100E3;

// temperature measure frequency
byte TEMPERATURE_MEASURE_FREQUENCY = 2;  // measure every 2nd wake-up

// pin definitions (GPIO0-GPIO15 all have internal pull-ups)
const int pinServoCtrl   = 4;  // GPIO04 servo control, D2 on D1 mini
const int pinServoPower  = 14; // GPIO14 servo power, D5 on D1 mini
const int pinManualOpen  = 5;  // GPIO05 connects key to manually open, D1 on D1 mini
const int pinManualClose = 12; // GPIO12 connects key to manually close, D6 on D1 mini
const int pinDHT22       = 13; // GPIO13 DHT22 data, D7 on D1 mini


// deep sleep period
const unsigned long SLEEP_PERIOD = 3600000000;


void callback(char* topic, byte* payload, unsigned int length);
int  processCmd(String cmd);
int  controlServo(int angle);


WiFiClient espClient;
PubSubClient client(espClient);
  
void setup() {
  // setup serial
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println();
  Serial.println("RabbitHatchDoor started");

  // check local push buttons for manual control
  bool localControl = false;
  int  newAngle = -1;
  char localCmd[10];

  pinMode(pinServoPower, OUTPUT);
  digitalWrite(pinServoPower,LOW);
  
  pinMode(pinManualOpen, INPUT_PULLUP);
  pinMode(pinManualClose, INPUT_PULLUP);
  delay(200);
    
  if(digitalRead(pinManualOpen) == LOW){ 
    Serial.println("push button manual open pressed");
    localControl = true;
    strcpy(localCmd,"open");
    newAngle = processCmd(localCmd);
  }
  if(digitalRead(pinManualClose) == LOW){ 
    Serial.println("push button manual close pressed");
    localControl = true;
    strcpy(localCmd,"close");
    newAngle = processCmd(localCmd);
  }
  
  // Connect to WiFi network
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to SID ");
  Serial.println(SSID);
  WiFi.begin(SSID, password);

  static int COUNTER_MAX = 20;
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED && counter<COUNTER_MAX) {
    delay(500);
    Serial.print(".");
    counter++;
  }

  if(counter>=COUNTER_MAX) {
    Serial.print("Connection failed - sleeping again");
    ESP.deepSleep(SLEEP_PERIOD);
  }
  Serial.println("");
  Serial.print("Connected to WiFi, IP address=");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  char buffer[128];
  sprintf(buffer,"%s-%d",mqtt_client,ESP.getChipId());
  
  Serial.print("Attempting MQTT connection to broker at ");
  Serial.print(mqtt_server);
  Serial.print(" as client ");
  Serial.println(buffer);
    
  // Attempt to connect
  if (client.connect(buffer)) {
    Serial.println("connected");
    // Once connected, publish an announcement...
    client.publish("rabbithutch/connect", "connected");

    if(localControl) {
      client.publish("rabbithutch/local", localCmd);

      // and publish new angle to MQTT broker
      if(newAngle>=0) {
        char buffer[200];
        sprintf(buffer,"publishing to topic %s : %d",topicAngle,newAngle);
        Serial.println(buffer);
        sprintf(buffer,"%d",newAngle);
        client.publish(topicAngle, buffer);
      }
    }
    else {
      // subscribe to opic and check for retained publications
      client.subscribe(topicDoor);
      client.subscribe(topicAngleOpen);
      client.subscribe(topicAngleClose);
    }

    // measure battery voltage and publish
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

    // now measure temperature
    /*
    EEPROM.begin(16);
    byte value = EEPROM.read(EEPROM_TEMPERATURE_COUNTER);
    if(value >= TEMPERATURE_MEASURE_FREQUENCY) {
      Serial.println("triggering temperature measure");
      readTemperature();
      value = 0;
    }
    value++;
    EEPROM.write(EEPROM_TEMPERATURE_COUNTER,value);
    EEPROM.end();
    */

    for(int i=0 ; i<20 ; i++) {
      client.loop();
      delay(100);      
    }
    
    client.disconnect();
    WiFi.disconnect();
  }
  else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
  }

  Serial.print("sleeping ");
  Serial.print(SLEEP_PERIOD/1000000UL);
  Serial.println("s");
  // wake-up of deep sleep mode requires connection between GPIO16 (D0 on Mini 1)
  // and RST and is actually a reset of the chip
  pinMode(D0, WAKEUP_PULLUP);
  ESP.deepSleep(SLEEP_PERIOD,RF_CAL);
}

void loop() {
}

    
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  char payloadString[length+1];
  strncpy(payloadString,(char*)payload,length);
  payloadString[length] = 0;
    
  if(strcmp(topic,topicDoor)==0) {
    int newAngle = processCmd(payloadString);

    if(newAngle >= 0 ) {
      // publish new angle to MQTT broker
      char buffer[200];
      sprintf(buffer,"publishing to topic %s : %d",topicAngle,newAngle);
      Serial.println(buffer);
      sprintf(buffer,"%d",newAngle);
      client.publish(topicAngle, buffer);
    }
  }
  
  if(strcmp(topic,topicAngleOpen)==0) {
    EEPROM.begin(4);
    byte angle = (byte)atoi(payloadString);
    Serial.print("recived angleOpen: ");
    Serial.println((int)angle);

    int oldAngle = EEPROM.read(EEPROM_ANGLE_OPEN);
    if(angle!=oldAngle) {
      EEPROM.write(EEPROM_ANGLE_OPEN,angle);
    }
    EEPROM.end();
  }
  
  if(strcmp(topic,topicAngleClose)==0) {
    EEPROM.begin(4);
    byte angle = (byte)atoi(payloadString);
    Serial.print("recived angleClose: ");
    Serial.println((int)angle);

    int oldAngle = EEPROM.read(EEPROM_ANGLE_CLOSE);
    if(angle!=oldAngle) {
      EEPROM.write(EEPROM_ANGLE_CLOSE,angle);
    }
    EEPROM.end();
  }
}

// returns -1 if position was not changes
// otherwise returns new angle
int processCmd(String cmd) {
  if(cmd == "nothing") {
    Serial.println("no action needed");
    return -1;
  }

  EEPROM.begin(4);
  if(cmd == "open") {
    int angle = EEPROM.read(EEPROM_ANGLE_OPEN);
    Serial.print("received door open, angle open=");
    Serial.println(angle);
    return controlServo(angle);
  }

  if(cmd == "close") {
    int angle = EEPROM.read(EEPROM_ANGLE_CLOSE);
    Serial.print("received door close, angle close=");
    Serial.println(angle);
    return controlServo(angle);
  }

  Serial.print("Unknown servo command: ");
  Serial.println(cmd);

  return -1;
}


// returns -1 if position was not changes
// otherwise returns new angle
int controlServo(int angle) { 
  int oldAngle = EEPROM.read(EEPROM_ANGLE_LAST);

  Serial.print("old angle: ");
  Serial.println(oldAngle);
  Serial.print("new angle: ");
  Serial.println(angle);

  if(oldAngle>180) {
    oldAngle = 180;
    Serial.println("correcting old angle to 180");
  }

  
  if(angle>180) {
    angle = 180;
    Serial.println("correcting angle to 180");
  }

  if(angle==oldAngle) {
    Serial.println("no change in door position");
    return -1;
  }

  Servo servo;
  servo.attach(pinServoCtrl);
  servo.write(oldAngle);
  delay(100);

  // make sue DHHT22 data pin is low during power anble
  // otherwise COM port at PC gets reset - GND shifts ?
  pinMode(pinDHT22, OUTPUT);
  digitalWrite(pinDHT22,LOW);
    
  digitalWrite(pinServoPower,HIGH);
      
  if(angle>oldAngle) {
    for(int i=oldAngle ; i<angle ; i+=5) {
      servo.write(i);
      delay(100);
    }
  }
  else {
    for(int i=oldAngle ; i>angle ; i-=5) {
      servo.write(i);
      delay(100);
    }
  }
  
  servo.write(angle);
  
  EEPROM.write(EEPROM_ANGLE_LAST,angle);  
  EEPROM.end();
  
  digitalWrite(pinServoPower,LOW);
  delay(500);

  return angle;
}

// read temperature on DHT22 (AM2302)
void readTemperature()
{
    // make sue DHHT22 data pin is low during power anble
    // otherwise COM port at PC gets reset - GND shifts ?
    pinMode(pinDHT22, OUTPUT);
    digitalWrite(pinDHT22,LOW);
    delay(100);

    digitalWrite(pinServoPower,HIGH);
    delay(500);
    
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
