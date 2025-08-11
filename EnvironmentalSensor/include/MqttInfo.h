/**
 * this file contains the definition of MQTT parameters
 */
#ifndef MQTT_INFO_H
#define MQTT_INFO_H

#define MQTT_SERVER "192.168.178.27"    // IP address of the MQTT broker
#define MQTT_PORT   1883                // port of the MQTT broker

#define MQTT_CLIENT_ID             "EnvironmentalSensor"  // client ID of the MQTT client
#define MQTT_PREFIX                "sensor/heatingroom/"  // prefix for all MQTT topics related to receiver

// MQTT publish topics
const char* topicPublishConnected             = MQTT_PREFIX "connected";       // gets published right after connect
const char* topicPublishIsAlive               = MQTT_PREFIX "isAlive";         // gets published peridocally to indicate that the device is still alive
const char* topicPublishIlluminance           = MQTT_PREFIX "illuminance";     // illuminance value (lux)
const char* topicPublishCoSensorVoltage       = MQTT_PREFIX "coVoltage";       // CO sensor voltage (V)
const char* topicPublishSensorTemperature     = MQTT_PREFIX "temperature";     // sensor temperature data
const char* topicPublishSensorHumidity        = MQTT_PREFIX "humidity";        // sensor humidity data
const char* topicPublishSensorIaq             = MQTT_PREFIX "iaq";             // sensor IAQ data
const char* topicPublishSensorIaqAccuracy     = MQTT_PREFIX "iaqAccuracy";     // sensor IAQ accuracy data
const char* topicPublishSensorCo2             = MQTT_PREFIX "co2";             // sensor CO2 data
const char* topicPublishSensorGasResistance   = MQTT_PREFIX "gasResistance";   // sensor gas resistance data (Ohms)

#endif