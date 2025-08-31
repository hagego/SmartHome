/**
 * this file contains the definition of MQTT parameters
 */
#ifndef MQTT_INFO_H
#define MQTT_INFO_H

#define MQTT_SERVER "192.168.178.27"    // IP address of the MQTT broker
#define MQTT_PORT   1883                // port of the MQTT broker

#define MQTT_CLIENT_ID "garagedoorserver"     // client ID of the MQTT client
#define MQTT_PREFIX    "garage/"              // prefix for all MQTT topics

// MQTT publish topics
const char* topicPublishConnected      = MQTT_PREFIX "connected";     // gets published right after connect
const char* topicPublishIsAlive        = MQTT_PREFIX "isAlive";       // gets published periodically
const char* topicPublishRssi           = MQTT_PREFIX "rssi";          // WIFi RSSI for tracking purposes

// MQTT subscription topics
const char* topicSubscribeCommand      = MQTT_PREFIX "door";           // command to control the door

#endif