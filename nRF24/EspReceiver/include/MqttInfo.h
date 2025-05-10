/**
 * this file contains the definition of MQTT parameters
 */
#ifndef MQTT_INFO_H
#define MQTT_INFO_H

#define MQTT_SERVER "192.168.178.27"    // IP address of the MQTT broker
#define MQTT_PORT   1883                // port of the MQTT broker

#define MQTT_CLIENT_ID "nRF24Receiver"     // client ID of the MQTT client
#define MQTT_PREFIX    "nRF24Receiver/"    // prefix for all MQTT topics

// MQTT publish topics
const char* topicPublishConnected       = MQTT_PREFIX "connected";       // gets published right after connect
const char* topicPublishPayloadReceived = MQTT_PREFIX "payloadReceived"; // gets published when a payload is received

// MQTT subscription topics
const char* topicSubscribeSetAddress      = MQTT_PREFIX "setAddress";         // set nRF24 address 0 (max. 5 bytes/characters)()

#endif