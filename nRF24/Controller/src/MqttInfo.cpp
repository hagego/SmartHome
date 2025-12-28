#include "MqttInfo.h"

// MQTT publish topics
const char* MqttInfo::topicPublishConnected       = MQTT_PREFIX "connected";        // gets published right after connect
const char* MqttInfo::topicPublishIsAlive         = MQTT_PREFIX "isAlive";          // gets published right after connect
const char* MqttInfo::topicPublishDebugMessage    = MQTT_PREFIX "debugMessage";     // debug messages (if enabled)
const char* MqttInfo::topicPublishClientMessage   = "nRF24Client/clientMessage";    // message from nRF24 clients

// MQTT subscription topics related to controller
const char* MqttInfo::topicSubscribeEnableMqttDebug = MQTT_PREFIX "enableMqttDebug";  // enable or disable debug messages (payload "1" or "0")
const char* MqttInfo::topicSubscribeClientCommand   = MQTT_PREFIX "clientCommand/+";  // command to send to a nRF24 client, suffix is the client ID

// MQTT subscription topics related to sensors
const char* MqttInfo::topicPublishSensorConnected       = "connected";      // sensor connected
const char* MqttInfo::topicPublishSensorBattery         = "battery"; // sensor battery voltage
const char* MqttInfo::topicPublishSensorMotionDetected  = "motionDetected"; // sensor motion detected
const char* MqttInfo::topicPublishSensorTemperature     = "temperature";    // sensor temperature data
const char* MqttInfo::topicPublishSensorHumidity        = "humidity";       // sensor humidity data
const char* MqttInfo::topicPublishSensorIaq             = "iaq";            // sensor IAQ data
const char* MqttInfo::topicPublishSensorIaqAccuracy     = "iaqAccuracy";    // sensor IAQ accuracy data
const char* MqttInfo::topicPublishSensorCo2             = "co2";            // sensor CO2 data
const char* MqttInfo::topicPublishSensorGasResistance   = "gasResistance";  // sensor gas resistance data (Ohms)
const char* MqttInfo::topicPublishSensorIlluminance     = "illuminance";    // sensor illuminance data

// array to map of RF24 client IDs to MQTT prefixes
const char* MqttInfo::mqttPrefixForClientId[NUM_CLIENT_PREFIXES] = {
    "sensor/livingroom/", // client ID 0 : locally connected sensor(s)
    "sensor/dg/",         // client ID 1 : battery light DG
    "sensor/entrance/",   // client ID 2 : motion sensor entrance
    "sensor/wallbox/",    // client ID 3 : battery light wallbox
    "sensor/terrace1/"    // client ID 4 : terrace sensor
};
