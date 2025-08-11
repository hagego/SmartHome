/*
  ESP8266 D1 code to measure environmental values using BH1750,BME680 and MQ-7 sensors and publish the value via MQTT broker.
*/
#include <Arduino.h>

#ifdef ESP32
#include <WiFi.h>
#include <AsyncTCP.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#endif
#include <ESPAsyncWebServer.h>
#include <ESPAsyncHTTPUpdateServer.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <BH1750.h>
#include <bsec2.h>
#include <Wire.h>




// include WLAN authentification information
#include "WifiInfo.h"

// include MQTT information
#include "MqttInfo.h"

#ifndef WIFI_SSID
#define WIFI_SSID "my_ssid"
#define WIFI_PSK  "my_psk"
#endif

// speed of serial interface for debug messages
#define SERIAL_SPEED 74880

// global objects
WiFiClient   wifiClient;
PubSubClient mqttClient(wifiClient);
Bsec2        envSensor;

ESPAsyncHTTPUpdateServer updateServer;
AsyncWebServer           webServer(80);
 
 // global buffer object for sprinf and other string operations
char         buffer[256];

// resistive divider values for CO sensor voltage measurement
const double ADC_RESISTOR_EXTERNAL = 220E3;  // external between sensor output and A0
const double ADC_RESISTOR_INTERNAL = 220E3;  // internal in ESP8266 module btw. A0 and ADC input
const double ADC_RESISTOR_MEASURE  = 100E3;  // internal in ESP8266 module btw. ADC and GND

// page not found handler for HTTP update server
void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}


// measure illuminance using BH1750 sensor
void readIlluminance() {
  Serial.println(F("readIlluminance()"));

  BH1750 lightMeter(0x23); // Address 0x23 is the default address of the sensor
  Serial.println(F("lighMeter created"));
  delay(10); // ensure enough time for Serial output in case of error

  if (lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE)) {
    Serial.println(F("BH1750 Advanced begin"));
    delay(1000); // ensure enough time for Serial output in case of error and measurement
    if (lightMeter.measurementReady(true)) {
      float lux = lightMeter.readLightLevel();
      sprintf(buffer,"publishing to topic %s : %.1f",topicPublishIlluminance,lux);
      Serial.println(buffer);
      sprintf(buffer,"%.1f",lux);
      mqttClient.publish(topicPublishIlluminance, buffer);
    } else {
      Serial.println(F("Error reading BH1750 light level"));
    }
  } else {
    Serial.println(F("Error initialising BH1750"));
  }
  delay(10); // ensure enough time for Serial output in case of error
}

// measure CO sensor voltage and publish
void measureVoltage() {
  Serial.println("measuring sensor voltage");
  
  int adcValue = analogRead(A0);
  sprintf(buffer,"ADC value: %d",adcValue);
  Serial.println(buffer);
  double v = ((double)adcValue/1023.0)*(ADC_RESISTOR_MEASURE+ADC_RESISTOR_INTERNAL+ADC_RESISTOR_EXTERNAL)/ADC_RESISTOR_MEASURE;
  sprintf(buffer,"voltage [V]: %.2f",v);
  Serial.println(buffer);

  sprintf(buffer,"%.2f",v);
  mqttClient.publish(topicPublishCoSensorVoltage, buffer);    
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
  
void setup() {
  // setup serial
  Serial.begin(SERIAL_SPEED);
  delay(100);
  Serial.println();
  Serial.println();
  Serial.println(F("environmental sensor started"));
  
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
    Serial.print(F("Connection failed - rebooting"));
    ESP.restart();
  }

  Serial.println("");
  Serial.print(F("Connected to WiFi, IP address="));
  Serial.println(WiFi.localIP());

  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setKeepAlive(300);   // set MQTT keep alive to 5 minutes

  char mqttFullClientName[128];
  sprintf(mqttFullClientName,"%s-%d",MQTT_CLIENT_ID,ESP.getChipId());

  sprintf(buffer,"attempting to connect to MQTT broker at %s as client %s",MQTT_SERVER,mqttFullClientName);
  Serial.println(buffer);
    
  // Attempt to connect
  if (mqttClient.connect(mqttFullClientName)) {
    // Once connected, publish an announcement...
    sprintf(buffer,"connected to MQTT broker at %s as client %s, local IP=%s",MQTT_SERVER,mqttFullClientName,WiFi.localIP().toString().c_str());
    Serial.println(buffer);
    mqttClient.publish(topicPublishConnected, WiFi.localIP().toString().c_str());
  }
  else {
    Serial.print("failed, rc=");
    Serial.print(mqttClient.state());
  }

  // setup the HTTP update server
  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/plain", "Sensor Heizungsraum, ready for updates. Use /update to upload new firmware.");
  });


  webServer.onNotFound(notFound);

  //setup the updateServer with credentials
  updateServer.setup(&webServer);

  webServer.begin();

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

  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin(); // uses default pins SDA=GPIO4=D2, SCL=GPIO5=D1
  Serial.println(F("Wire.begin() done"));
  delay(10); // ensure enough time for Serial output in case of error

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
}

int counter = 0;
void loop() {
  Serial.println(F("loop iteration starting"));

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
        mqttClient.publish(topicPublishConnected, WiFi.localIP().toString().c_str());
      }
    }
    else {
      Serial.println("WIFI reconnect failed. Rebooting...");
      // reboot
      ESP.restart();
    }
  }
  Serial.println(F("WIFI connected"));

  if(!mqttClient.connected()) {
    Serial.println(F("MQTT connection lost - restarting"));
    ESP.restart();
  }
  mqttClient.loop();

  // send data to broker every 5 iterations (=5 minutes)
  if(++counter == 5) {
    counter = 0;

    // send alive signal
    mqttClient.publish(topicPublishIsAlive, WiFi.localIP().toString().c_str());

    // read illuminance
    readIlluminance();

    // measure CO sensor voltage
    measureVoltage();
  }

  if (!envSensor.run())
  {
    Serial.println(F("BME .run() failed - checking status"));
    checkBsecStatus(envSensor);
  }

  Serial.println(F("sleeping for 1min"));
  delay(60000); // 1 min
}


