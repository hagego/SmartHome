#include "MqttInfo.h"

// MQTT publish topics
const char* MqttInfo::topicPublishConnected       = MQTT_PREFIX "connected";        // gets published right after connect
const char* MqttInfo::topicPublishIsAlive         = MQTT_PREFIX "isAlive";          // gets published right after connect
const char* MqttInfo::topicPublishDebugMessage    = MQTT_PREFIX "debugMessage";     // debug messages (if enabled)
const char* MqttInfo::topicPublishClientMessage   = MQTT_PREFIX "clientMessage";    // messages from nRF24 clients

// MQTT subscription topics
const char* MqttInfo::topicSubscribeEnableMqttDebug = MQTT_PREFIX "enableMqttDebug";  // enable or disable debug messages (payload "1" or "0")
const char* MqttInfo::topicSubscribeDebugCommand    = MQTT_PREFIX "debugCommand";     // debug command
const char* MqttInfo::topicSubscribeClientCommand   = MQTT_PREFIX "clientCommand/+";  // command to send to a nRF24 client, suffix is the client ID

