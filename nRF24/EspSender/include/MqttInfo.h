/**
 * this file contains the definition of MQTT parameters
 */
#ifndef MQTT_INFO_H
#define MQTT_INFO_H

#define MQTT_SERVER "192.168.178.27"    // IP address of the MQTT broker
#define MQTT_PORT   1883                // port of the MQTT broker

#define MQTT_CLIENT_ID "nRF24TestSender"     // client ID of the MQTT client
#define MQTT_PREFIX    "nRF24TestSender/"    // prefix for all MQTT topics

// MQTT publish topics
const char* topicPublishConnected      = MQTT_PREFIX "connected";     // gets published right after connect

// MQTT subscription topics
const char* topicSubscribeSetAddress      = MQTT_PREFIX "setAddress";         // set nRF24 address (max. 5 bytes/characters)()
const char* topicSubscribeTransmitPayload = MQTT_PREFIX "transmitPayload";    // transmits the specified payload (max. 32 bytes)

#endif