/**
 * this file contains the definition of MQTT parameters
 */
#ifndef MQTT_INFO_H
#define MQTT_INFO_H

#define MQTT_SERVER "192.168.178.27"    // IP address of the MQTT broker
#define MQTT_PORT   1883                // port of the MQTT broker

#define MQTT_CLIENT_ID      "nRF24Receiver"              // client ID of the MQTT client
#define MQTT_PREFIX         "nRF24Receiver/"             // prefix for all MQTT topics related to receiver
#define MQTT_PREFIX_SENSOR  "sensor/livingroom/"         // prefix for all MQTT topics related to sensor measurements
#define MQTT_PREFIX_MOTION1 "motionsensor/entrance/"     // prefix for all MQTT topics related to motion detection on motion sensor 1

// MQTT publish topics
const char* topicPublishConnected       = MQTT_PREFIX "connected";       // gets published right after connect
const char* topicPublishIsAlive         = MQTT_PREFIX "isAlive";         // gets published right after connect
const char* topicPublishPayloadReceived = MQTT_PREFIX "payloadReceived"; // gets published when a payload is received

// MQTT motion detection publish topics
const char* topicPublishMotion1Detected    = MQTT_PREFIX_MOTION1 "motionDetected"; // gets published when motion is detected by motion sensor 1
const char* topicPublishMotion1Voltage     = MQTT_PREFIX_MOTION1 "battery";        // gets published when motion sensor 1 battery voltage is read
const char* topicPublishMotion1Illuminance = MQTT_PREFIX_MOTION1 "illuminance";    // gets published when motion sensor 1 illuminance is read

// MQTT sensor data publish topics
const char* topicPublishSensorTemperature  = MQTT_PREFIX_SENSOR "temperature";   // sensor temperature data
const char* topicPublishSensorHumidity     = MQTT_PREFIX_SENSOR "humidity";      // sensor humidity data
const char* topicPublishSensorIaq          = MQTT_PREFIX_SENSOR "iaq";           // sensor IAQ data
const char* topicPublishSensorIaqAccuracy  = MQTT_PREFIX_SENSOR "iaqAccuracy";   // sensor IAQ accuracy data
const char* topicPublishSensorCo2          = MQTT_PREFIX_SENSOR "co2";           // sensor CO2 data

// MQTT subscription topics
const char* topicSubscribeSetAddress      = MQTT_PREFIX "setAddress";         // set nRF24 address 0 (max. 5 bytes/characters)()

#endif