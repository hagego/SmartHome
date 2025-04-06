/**
 * this file contains the definition of MQTT parameters
 */
#ifndef MQTT_INFO_H
#define MQTT_INFO_H

#define MQTT_SERVER "192.168.178.27"    // IP address of the MQTT broker
#define MQTT_PORT   1883                // port of the MQTT broker

#define MQTT_CLIENT_ID "testsensor"     // client ID of the MQTT client
#define MQTT_PREFIX    "testsensor/"    // prefix for all MQTT topics

// MQTT publish topics
const char* topicPublishConnected      = MQTT_PREFIX "connected";     // gets published right after connect
const char* topicPublishTemperature    = MQTT_PREFIX "temperature";   // temperature
const char* topicPublishHumidity       = MQTT_PREFIX "humidity";      // humidity
const char* topicPublishBattery        = MQTT_PREFIX "battery";       // battery voltage

// MQTT subscription topics
const char* topicSubscribeSleepTime    = MQTT_PREFIX "sleepTime";      // sleep time [s]
const char* topicSubscribeAngleOpen    = MQTT_PREFIX "angleOpen";      // angle for servo open command
const char* topicSubscribeAngleClose   = MQTT_PREFIX "angleClose";     // angle for servo close command
const char* topicSubscribeServoCommand = MQTT_PREFIX "servoCommand";   // servo command

#endif