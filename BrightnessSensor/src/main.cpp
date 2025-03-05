/*
  ESP8266 code to measure and report the voltage of a solar cell
*/

#undef  MEASURE_VBAT
#define MEASURE_BRIGHTNESS
#undef  MEASURE_TEMP

#define MQTT_PREFIX "brightnesssensor/terrace/"

// MQTT client name
const char* mqtt_client = "brightnesssensor";

#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#ifdef MEASURE_BRIGHTNESS
#include <BH1750.h>
#include <Wire.h>
#endif

#ifdef MEASURE_TEMP
#include <DHT.h>
#endif

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
const char* topicAlive            = MQTT_PREFIX "alive";
const char* topicTemperature      = MQTT_PREFIX "temperature";
const char* topicVbat             = MQTT_PREFIX "pvcellVoltage";
const char* topicBrightness       = MQTT_PREFIX "brightness";


// resistor values for battery voltage measurement
//const double ADC_RESISTOR_EXTERNAL = 1E6;
const double ADC_RESISTOR_EXTERNAL = 470E3;
const double ADC_RESISTOR_INTERNAL = 220E3;
const double ADC_RESISTOR_MEASURE  = 100E3;

// pin definitions (GPIO0-GPIO15 all have internal pull-ups)
const int pinPower  = 14; // GPIO14 servo power, D5 on D1 mini
const int pinDHT22  = 13; // GPIO13 DHT22 data, D7 on D1 mini


// deep sleep period
unsigned long SLEEP_PERIOD = 1800000000; // unit is us => 30min


// code found on https://github.com/esp8266/Arduino/issues/6318
// ESP.deepSleep() not workin reliable on all modules
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

WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);


// measure brightness using BH1750 sensor
#ifdef MEASURE_BRIGHTNESS
void readBrightness() {
  char buffer[128];

  Serial.println(F("readBrightness()"));
  delay(10); // ensure enough time for Serial output in case of error

  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin(); // uses default pins SDA=GPIO4=D2, SCL=GPIO5=D1

  Serial.println(F("Wire.begin() done"));
  delay(10); // ensure enough time for Serial output in case of error

  BH1750 lightMeter(0x23); // Address 0x23 is the default address of the sensor
  Serial.println(F("lighMeter created"));
  delay(10); // ensure enough time for Serial output in case of error

  if (lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE)) {
    Serial.println(F("BH1750 Advanced begin"));
    delay(1000); // ensure enough time for Serial output in case of error and measurement
    if (lightMeter.measurementReady(true)) {
      float lux = lightMeter.readLightLevel();
      sprintf(buffer,"publishing to topic %s : %f",topicBrightness,lux);
      Serial.println(buffer);
      sprintf(buffer,"%f",lux);
      mqttClient.publish(topicBrightness, buffer);
    } else {
      Serial.println(F("Error reading BH1750 light level"));
    }
  } else {
    Serial.println(F("Error initialising BH1750"));
  }
  delay(10); // ensure enough time for Serial output in case of error
}
#endif


// measure volate ad A0 pin and publish to MQTT broker
void measureVbat() {
  char buffer[128];

  Serial.println(F("measureVbat()"));
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
    mqttClient.publish(topicVbat, buffer);
  }
}

  
void setup() {
  // set up pins
  pinMode(pinPower, OUTPUT);
  digitalWrite(pinPower,LOW);

  pinMode(D0, WAKEUP_PULLUP);

  // setup serial
  Serial.begin(SERIAL_SPEED);
  delay(100);
  Serial.println();
  Serial.println();
  Serial.println(F("brightness sensor started"));
  
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
    Serial.print(F("Connection failed - sleeping again"));

    //ESP.deepSleep(SLEEP_PERIOD);
    DeepSleepNK(SLEEP_PERIOD);
  }

  Serial.println("");
  Serial.print(F("Connected to WiFi, IP address="));
  Serial.println(WiFi.localIP());

  mqttClient.setServer(mqtt_server, 1883);

  char buffer[256];
  char mqttFullClientName[128];
  sprintf(mqttFullClientName,"%s-%d",mqtt_client,ESP.getChipId());

  sprintf(buffer,"attempting to connect to MQTT broker at %s as client %s",mqtt_server,mqttFullClientName);
  Serial.println(buffer);
    
  // Attempt to connect
  if (mqttClient.connect(mqttFullClientName)) {
    // Once connected, publish an announcement...
    sprintf(buffer,"connected to MQTT broker at %s as client %s, local IP=%s",mqtt_server,mqttFullClientName,WiFi.localIP().toString().c_str());
    Serial.println(buffer);
    mqttClient.publish(topicConnected, buffer);

    #ifdef MEASURE_VBAT
    measureVbat();
    #endif

    // now measure temperature
    #ifdef MEASURE_TEMP
    Serial.println(F("triggering temperature measure"));
    readTemperature();
    #endif

    // now measure brightness
    #ifdef MEASURE_BRIGHTNESS
    readBrightness();
    #endif
    
    mqttClient.disconnect();
    WiFi.disconnect();
  }
  else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
  }

  digitalWrite(pinPower,LOW);

  delay(1000);
  
  Serial.print("sleeping ");
  Serial.print(SLEEP_PERIOD/1000000UL);
  Serial.println("s");

  // wake-up of deep sleep mode requires connection between GPIO16 (D0 on Mini 1)
  // and RST and is actually a reset of the chip
  ESP.deepSleep(SLEEP_PERIOD,WAKE_RF_DISABLED);

  //DeepSleepNK(SLEEP_PERIOD);
}

void loop() {
  // we should never get here
}

// read temperature on DHT22 (AM2302)
#ifdef MEASURE_TEMP
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

