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
const char* topicTemperature   = "rabbitfeeder/temperature";
const char* topicHatchCmd      = "rabbitfeeder/hatchCmd";
const char* topicConnect       = "rabbitfeeder/connect";

const int ANGLE_OPEN   = 100;
const int ANGLE_CLOSED = 20;

// pin definitions (GPIO0-GPIO15 all have internal pull-ups)
const int pinServoCtrl   = 4;  // GPIO04 servo control, D2 on D1 mini
const int pinServoPower  = 14; // GPIO14 servo power, D5 on D1 mini
const int pinDHT22       = 13; // GPIO13 DHT22 data, D7 on D1 mini


// deep sleep period
const unsigned long SLEEP_PERIOD = 3600000000; // 3600 seconds = 1h
const unsigned long FEED_PERIOD  = 2000000;    // 5 seconds

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
  
  delay(200);
    
  // Connect to WiFi network
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to SID ");
  Serial.println(SSID);
  WiFi.begin(SSID, password);

  const int COUNTER_MAX = 40;
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

  client.setServer(mqttServer, 1883);
  client.setCallback(callback);
  
  Serial.print("Attempting MQTT connection to broker at ");
  Serial.println(mqttServer);
  // Attempt to connect
  if (client.connect(mqttClientName)) {
    Serial.println("MQTT connected");
    // Once connected, publish an announcement...
    client.publish(topicConnect, "connected");

    // subscribe for commands for hatch control
    client.subscribe(topicHatchCmd);

    // MQTT client processing loop
    for(int i=0 ; i<20 ; i++) {
      client.loop();
      delay(100);      
    }

    DHT dht(pinDHT22,DHT22);
    dht.begin();
    delay(2000);
    float t = dht.readTemperature();
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
}

// open the hatch door
void openHatch() {
  Servo servo;

  digitalWrite(pinServoPower,HIGH);
  servo.attach(pinServoCtrl);
  
  for(int angle=ANGLE_CLOSED ; angle<=ANGLE_OPEN ; angle += 10) {
    servo.write(angle);
    delay(100);
  }  

  delay(FEED_PERIOD/1000);

  for(int angle=ANGLE_OPEN ; angle>=ANGLE_CLOSED ; angle -= 10) {
    servo.write(angle);
    delay(100);
  }  
  
  client.publish(topicHatchCmd, "done",true);
}
