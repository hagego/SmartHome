/**
 * contains code to measure temperature and battery voltage
 */
#ifndef MEASURE_H
#define MEASURE_H

#include <PubSubClient.h>
#include <DHTesp.h>
#include "MqttInfo.h"

// resistor values for battery voltage measurement
const double ADC_RESISTOR_EXTERNAL =   1E6;
const double ADC_RESISTOR_INTERNAL = 220E3;
const double ADC_RESISTOR_MEASURE  = 100E3;

// measure temperature and humidity
void measureTemperature(u_int8_t pinDHT22, PubSubClient& mqttClient) {
    char buffer[128];

    DHTesp dht;
    dht.setup(pinDHT22, DHTesp::DHT22);
    delay(dht.getMinimumSamplingPeriod()); // wait for sensor to stabilize
    float t = dht.getTemperature();
    float h = dht.getHumidity();
  
    sprintf(buffer,"DHT22 measurement completed. Temperature=%.1f Humidity=%.1f",t,h);
    Serial.println(buffer);

    // check if temperature is valid
    if(isnan(t)) {
        Serial.println(F("temperature reading is nan - doing nothing"));
    }
    else {
        sprintf(buffer,"publishing to topic %s : %.1f",topicPublishTemperature,t);
        Serial.println(buffer);
        sprintf(buffer,"%.1f",t);
        mqttClient.publish(topicPublishTemperature, buffer);
    }
    
    // check if humidity is valid
    if(isnan(h)) {
        Serial.println(F("humidity reading is nan - doing nothing"));
    }
    else {
        sprintf(buffer,"publishing to topic %s : %.1f",topicPublishHumidity,h);
        Serial.println(buffer);
        sprintf(buffer,"%.1f",h);
        mqttClient.publish(topicPublishHumidity, buffer);
    }
}

// measure battery voltage
void measureBatteryVoltage(PubSubClient& mqttClient) {
    char buffer[128];

    // measure battery voltage and publish
    int adcValue = analogRead(A0);
    double vbat = ((double)adcValue/1023.0)*(ADC_RESISTOR_MEASURE+ADC_RESISTOR_INTERNAL+ADC_RESISTOR_EXTERNAL)/ADC_RESISTOR_MEASURE;
    sprintf(buffer,"ADC value: %d battery voltage=%.1f V",adcValue,vbat);
    Serial.println(buffer);
        
    if(adcValue<10) {
        // no voltage measured, most likely chip is not on the application PCB
        // do nothing
        Serial.println(F("assuming chip is not on application board - doing nothing"));
    }
    else {
        sprintf(buffer,"publishing to topic %s : %.1f",topicPublishBattery,vbat);
        Serial.println(buffer);
        sprintf(buffer,"%.1f",vbat);
        mqttClient.publish(topicPublishBattery, buffer);
    }
}

#endif
