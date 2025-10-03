/**
 * this file contains the definition of MQTT parameters
 */
#ifndef MQTT_INFO_H
#define MQTT_INFO_H

#define MQTT_SERVER "192.168.178.27"    // IP address of the MQTT broker
#define MQTT_PORT   1883                // port of the MQTT broker

#define MQTT_CLIENT_ID             "nRF24RemoteControl"        // client ID of the MQTT client
#define MQTT_PREFIX                "nRF24RemoteControl/"       // prefix for all MQTT topics related to remote control
#define MQTT_PREFIX_LOCAL_SENSOR   "sensor/livingroom/"   // prefix for all MQTT topics related to sensor measurements
#define MQTT_PREFIX_REMOTE_SENSOR1 "sensor/debug/"     // prefix for all MQTT topics related to motion detection on motion sensor 1

// MQTT publish topics
const char* topicPublishConnected       = MQTT_PREFIX "connected";       // gets published right after connect
const char* topicPublishIsAlive         = MQTT_PREFIX "isAlive";         // gets published right after connect
const char* topicPublishPayloadReceived = MQTT_PREFIX "payloadReceived"; // gets published when a payload is received

// MQTT remote sensor 1 publish topics
const char* topicPublishRemoteSensor1Debug            = MQTT_PREFIX_REMOTE_SENSOR1 "debug";          // gets published whenever remote sensor 1 transmits something
const char* topicPublishRemoteSensor1Connected        = MQTT_PREFIX_REMOTE_SENSOR1 "connected";      // gets published when remote sensor 1 gets connected
const char* topicPublishRemoteSensor1MotionDetected   = MQTT_PREFIX_REMOTE_SENSOR1 "motionDetected"; // gets published when motion is detected by motion sensor 1
const char* topicPublishRemoteSensor1Voltage          = MQTT_PREFIX_REMOTE_SENSOR1 "battery";        // gets published when motion sensor 1 battery voltage is read
const char* topicPublishRemoteSensor1Illuminance      = MQTT_PREFIX_REMOTE_SENSOR1 "illuminance";    // gets published when motion sensor 1 illuminance is read

// MQTT local sensor data publish topics
const char* topicPublishSensorMotionDetected  = MQTT_PREFIX_LOCAL_SENSOR "motionDetected"; // local sensor motion detected
const char* topicPublishSensorTemperature     = MQTT_PREFIX_LOCAL_SENSOR "temperature";    // sensor temperature data
const char* topicPublishSensorHumidity        = MQTT_PREFIX_LOCAL_SENSOR "humidity";       // sensor humidity data
const char* topicPublishSensorIaq             = MQTT_PREFIX_LOCAL_SENSOR "iaq";            // sensor IAQ data
const char* topicPublishSensorIaqAccuracy     = MQTT_PREFIX_LOCAL_SENSOR "iaqAccuracy";    // sensor IAQ accuracy data
const char* topicPublishSensorCo2             = MQTT_PREFIX_LOCAL_SENSOR "co2";            // sensor CO2 data
const char* topicPublishSensorGasResistance   = MQTT_PREFIX_LOCAL_SENSOR "gasResistance";  // sensor gas resistance data (Ohms)

// MQTT subscription topics
const char* topicSubscribeSetAddress      = MQTT_PREFIX "setAddress";         // set nRF24 address 0 (max. 5 bytes/characters)()

#endif