/*
  ESP8266 code to control a door of a rabbit hatch based on periodically
  querying an openhab server for commands
*/

#define MEASURE_VBAT
#define MEASURE_TEMP
//#define MQTT_PREFIX "tempsensor/terrace/"
#define MQTT_PREFIX "rabbithutchdoor/"

// MQTT client name
//const char* mqtt_client = "tempsensorterrace";
const char* mqtt_client = "rabbithutchdoor";

#include <math.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>
#include <DHT.h>

// include WLAN authentification information/home/hagen/Maker/repos/SmartHome/RabbitHatchDoor/RabbitHatchDoor/WiFiInfo.h
#include "WifiInfo.h"

#ifndef WIFI_SSID
#define WIFI_SSID "my_ssid"
#define WIFI_PSK  "my_psk"
#endif

// speed of serial interface for debug messages
#define SERIAL_SPEED 74880

// MQTT broker IP address
const char* mqtt_server = "192.168.178.27";

// MQTT topics
// publish
const char* topicConnected        = MQTT_PREFIX "connected";
const char* topicDoorCmdReceived  = MQTT_PREFIX "doorCmdReceived";
const char* topicTemperature      = MQTT_PREFIX "temperature";
const char* topicVbat             = MQTT_PREFIX "vbat";
const char* topicAngle            = MQTT_PREFIX "angle";
const char* topicAngleLocal       = MQTT_PREFIX "local";
const char* topicDoorStatus       = MQTT_PREFIX "doorStatus";

// subscribed
const char* topicSubscribedDoor        = MQTT_PREFIX "door";
const char* topicSubscribedAngleOpen   = MQTT_PREFIX "angleOpen";
const char* topicSubscribedAngleClose  = MQTT_PREFIX "angleClose";
const char* topicSubscribedSleepPeriod = MQTT_PREFIX "sleepPeriod";

// EEPROM addresses
const int EEPROM_SIZE                = 1024;

const int EEPROM_ANGLE_LAST          = 0;
const int EEPROM_ANGLE_OPEN          = 1;
const int EEPROM_ANGLE_CLOSE         = 2;

// resistor values for battery voltage measurement
//const double ADC_RESISTOR_EXTERNAL = 1E6;
const double ADC_RESISTOR_EXTERNAL =   1E6;
const double ADC_RESISTOR_INTERNAL = 220E3;
const double ADC_RESISTOR_MEASURE  = 100E3;

// pin definitions (GPIO0-GPIO15 all have internal pull-ups)
const int pinServoCtrl   = 4;  // GPIO04 servo control, D2 on D1 mini
const int pinServoPower  = 14; // GPIO14 servo power, D5 on D1 mini
const int pinManualOpen  = 5;  // GPIO05 connects key to manually open, D1 on D1 mini
const int pinManualClose = 12; // GPIO12 connects key to manually close, D6 on D1 mini
const int pinDHT22       = 13; // GPIO13 DHT22 data, D7 on D1 mini


// default deep sleep period
// can be overridden thru MQTT topic topicSubscribedSleepPeriod
unsigned long SLEEP_PERIOD = 3600000000; // unit is us => 1h


void callback(char* topic, byte* payload, unsigned int length);
int  processCmd(String cmd);
int  controlServo(byte angle);

boolean servoChanged = false;
String  doorStatus;


// code found on https://github.com/esp8266/Arduino/issues/6318
// ESP.deepSleep() not workin reliable on all modules
uint32_t*RT= (uint32_t *)0x60000700;

void DeepSleepNK(uint64 t_us)
{
  //Serial.println(F("Entering DeepSleep"));
  //delay(500);

  //wifi_set_sleep_type(MODEM_SLEEP_T);
  //system_deep_sleep_set_option(2); // no RF calibration after wake-up
  //system_deep_sleep_instant(t_us);
  //Serial.println(F("after system_deep_sleep"));
  //delay(500);

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

PubSubClient* pClient = 0L;
  
void setup() {
  // check local push buttons for manual control
  bool localControl = false;
  int  newAngle = -1;
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

  pinMode(pinDHT22, INPUT_PULLUP);
  pinMode(pinServoPower, OUTPUT);
  digitalWrite(pinServoPower,LOW);

  pinMode(D0, WAKEUP_PULLUP);

  // setup serial
  Serial.begin(SERIAL_SPEED);
  delay(100);
  Serial.println();
  Serial.println();
  Serial.println(F("RabbitHatchDoor started"));

  EEPROM.begin(EEPROM_SIZE);

  if(localControl) {
    Serial.print(F("push button pressed, cmd="));
    Serial.println(localCmd);
    newAngle = processCmd(localCmd);
  }
  
  // Connect to WiFi network
  //WiFi.disconnect();
  WiFi.mode(WIFI_STA);
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
    Serial.print(F("Connection failed - committing EEPROM data and sleeping again"));
    EEPROM.end();

    //ESP.deepSleep(SLEEP_PERIOD);
    DeepSleepNK(SLEEP_PERIOD);
  }

  Serial.println("");
  Serial.print(F("Connected to WiFi, IP address="));
  Serial.println(WiFi.localIP());

  WiFiClient espClient;
  PubSubClient client(espClient);
  pClient = &client;

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  char buffer[128];
  sprintf(buffer,"%s-%d",mqtt_client,ESP.getChipId());
  
  Serial.print(F("Attempting MQTT connection to broker at "));
  Serial.print(mqtt_server);
  Serial.print(" as client ");
  Serial.println(buffer);
    
  // Attempt to connect
  if (client.connect(buffer)) {
    Serial.println(F("connected"));
    // Once connected, publish an announcement...
    client.publish(topicConnected, "connected");

    if(localControl) {
      client.publish(topicAngleLocal, localCmd);

      // and publish new angle to MQTT broker
      if(newAngle>=0) {
        sprintf(buffer,"publishing to topic %s : %d",topicAngle,newAngle);
        Serial.println(buffer);
        sprintf(buffer,"%d",newAngle);
        client.publish(topicAngle, buffer);
      }
    }
    else {
      // subscribe to topics and check for retained publications
      client.subscribe(topicSubscribedDoor);
      client.subscribe(topicSubscribedAngleOpen);
      client.subscribe(topicSubscribedAngleClose);
      client.subscribe(topicSubscribedSleepPeriod);
    }

    #ifdef MEASURE_VBAT
    // measure battery voltage and publish
    delay(100);
    int adcValue = analogRead(A0);
    Serial.print(F("ADC value: "));
    Serial.println(adcValue);

    if(adcValue<10) {
      // no voltage measured, most likely chip is not on the application PCB
      // do nothing
      Serial.println(F("assuming chip is not on application board - doing nothing"));
    }
    else {
      double vbat = ((double)adcValue/1023.0)*(ADC_RESISTOR_MEASURE+ADC_RESISTOR_INTERNAL+ADC_RESISTOR_EXTERNAL)/ADC_RESISTOR_MEASURE;
      Serial.print(F("battery voltage [V]: "));
      Serial.println(vbat);
      sprintf(buffer,"publishing to topic %s : %f",topicVbat,vbat);
      Serial.println(buffer);
      sprintf(buffer,"%f",vbat);
      client.publish(topicVbat, buffer);
    }
    #endif

    // now measure temperature
    #ifdef MEASURE_TEMP
    Serial.println(F("triggering temperature measure"));
    readTemperature();
    #endif

    // give enough time to receive MQTT publications
    for(int i=0 ; i<20 ; i++) {
      client.loop();
      delay(100);      
    }

    if(servoChanged) {
      client.publish(topicDoorStatus, doorStatus.c_str());
    }
    
    client.disconnect();
    WiFi.disconnect();
  }
  else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
  }

  Serial.println(F("committing EEPROM data"));
  EEPROM.end();

  digitalWrite(pinServoPower,LOW);
  
  Serial.print("sleeping ");
  Serial.print(SLEEP_PERIOD/1000000UL);
  Serial.println("s");

  // ensure EEPROM data could be written
  delay(500);

  // wake-up of deep sleep mode requires connection between GPIO16 (D0 on Mini 1)
  // and RST and is actually a reset of the chip
  //ESP.deepSleep(SLEEP_PERIOD,WAKE_RF_DISABLED);

  DeepSleepNK(SLEEP_PERIOD);
}

void loop() {
}

    
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] value=");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  char payloadString[length+1];
  strncpy(payloadString,(char*)payload,length);
  payloadString[length] = 0;
    
  if(strcmp(topic,topicSubscribedDoor)==0) {
    pClient->publish(topicDoorCmdReceived, payloadString);

    // clear retained message
    if(length>0) {
      pClient->publish(topicSubscribedDoor,"",true);
    }
    
    int newAngleRemote = processCmd(payloadString);

    if(newAngleRemote >= 0 ) {
      // publish new angle to MQTT broker
      char buffer[200];
      sprintf(buffer,"publishing to topic %s : %d",topicAngle,newAngleRemote);
      Serial.println(buffer);
      sprintf(buffer,"%d",newAngleRemote);
      pClient->publish(topicAngle, buffer);
    }
  }
  
  if(strcmp(topic,topicSubscribedAngleOpen)==0) {
    byte angle = (byte)atoi(payloadString);
    Serial.print("recived angleOpen: ");
    Serial.println(angle);

    byte oldAngle = EEPROM.read(EEPROM_ANGLE_OPEN);
    Serial.print(F("old angleOpen in EEPROM: "));
    Serial.println(oldAngle);
    if(angle!=oldAngle) {
      Serial.println("Storing angleOpen in EEPROM");
      EEPROM.write(EEPROM_ANGLE_OPEN,angle);
      bool commit=EEPROM.commit();
      Serial.print(F("EEPROM commit result: "));
      Serial.println(commit);
    }
  }
  
  if(strcmp(topic,topicSubscribedAngleClose)==0) {
    byte angle = (byte)atoi(payloadString);
    Serial.print("recived angleClose: ");
    Serial.println((int)angle);

    byte oldAngle = EEPROM.read(EEPROM_ANGLE_CLOSE);
    Serial.print("old angleClose in EEPROM: ");
    Serial.println(oldAngle);
    if(angle!=oldAngle) {
      Serial.println("Storing angleClose in EEPROM");
      EEPROM.write(EEPROM_ANGLE_CLOSE,angle);
      bool commit=EEPROM.commit();
      Serial.print(F("EEPROM commit result: "));
      Serial.println(commit);
    }
  }

  // check for override of sleep period
  if(strcmp(topic,topicSubscribedSleepPeriod)==0) {
    byte sleepPeriodMinutes = (byte)atoi(payloadString);
    Serial.print("recived sleepPeriod: ");
    Serial.println((int)sleepPeriodMinutes);

    SLEEP_PERIOD = (unsigned long)sleepPeriodMinutes*60L*1000000L;
  }
  
}

// returns -1 if position was not changes
// otherwise returns new angle
int processCmd(String cmd) {
  if(cmd == "") {
    Serial.println(F("clearing retained message"));
    pClient->unsubscribe(topicSubscribedDoor);
    return -1;
  }

  if(cmd == "nothing") {
    Serial.println(F("no action needed"));
    return -1;
  }

  if(cmd == "open") {
    byte angle = EEPROM.read(EEPROM_ANGLE_OPEN);
    Serial.print(F("received door open, angle open="));
    Serial.println(angle);
    doorStatus = "open";
    return controlServo(angle);
  }

  if(cmd == "close") {
    byte angle = EEPROM.read(EEPROM_ANGLE_CLOSE);
    Serial.print(F("received door close, angle close="));
    Serial.println(angle);
    doorStatus = "closed";
    return controlServo(angle);
  }

  Serial.print(F("Unknown servo command: "));
  Serial.println(cmd);

  return -1;
}


// returns -1 if position was not changed
// otherwise returns new angle
// if angle==0, only old position is programmed
int controlServo(byte angle) {
  Servo servo;

  byte oldAngle = EEPROM.read(EEPROM_ANGLE_LAST);

  Serial.print("old angle read from EEPROM: ");
  Serial.println(oldAngle);
  Serial.print("new angle: ");
  Serial.println(angle);

  if(oldAngle>180) {
    oldAngle = 180;
    Serial.println(F("correcting old angle to 180"));
  }

  
  if(angle>180) {
    angle = 180;
    Serial.println(F("correcting angle to 180"));
  }

  servo.attach(pinServoCtrl);
  servo.write(oldAngle);

  if(angle==0 || angle==oldAngle) {
    Serial.println(F("no change in door position"));
    return -1;
  }
  delay(100);
    
  digitalWrite(pinServoPower,HIGH);
  delay(200);
      
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
  servoChanged = true;

  Serial.print(F("storing angle in EEPROM: "));
  Serial.println(angle);
  EEPROM.write(EEPROM_ANGLE_LAST,angle);  
  
  return angle;
}

// read temperature on DHT22 (AM2302)
#ifdef MEASURE_VBAT
void readTemperature()
{
    // ensure servo position is programmed to old value before power is turned on
    controlServo(0);

    digitalWrite(pinServoPower,HIGH);
    delay(1000);
    
    DHT dht(pinDHT22,DHT22); 
    dht.begin();
    delay(100);

    float t = dht.readTemperature();

    if(isnan(t)) {
      Serial.println(F("temperature reading is nan - doing nothing"));
    }
    else {
      Serial.print("temperature: ");
      Serial.println(t);

      // and publish to MQTT broker
      char buffer[10];
      sprintf(buffer,"%.1f",t);
      pClient->publish(topicTemperature, buffer);
      Serial.print("publishing to topic ");
      Serial.print(topicTemperature);
      Serial.print(": ");
      Serial.println(buffer);
    }
}
#endif
