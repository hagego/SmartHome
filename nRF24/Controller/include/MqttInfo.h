/**
 * this file contains the definition of MQTT parameters
 */
#ifndef MQTT_INFO_H
#define MQTT_INFO_H

#define MQTT_SERVER "192.168.178.27"    // IP address of the MQTT broker
#define MQTT_PORT   1883                // port of the MQTT broker

class MqttInfo {
    public:
        // MQTT publish topics related to controller
        static const char* topicPublishConnected;
        static const char* topicPublishIsAlive;
        static const char* topicPublishDebugMessage;
        static const char* topicPublishClientMessage;

        // array to map of RF24 client IDs to MQTT prefixes
        static const int   NUM_CLIENT_PREFIXES = 3;
        static const char* mqttPrefixForClientId[];

        // MQTT publish topics related to sensors
        static const char* topicPublishSensorConnected;      // sensor connected
        static const char* topicPublishSensorBattery;        // sensor battery voltage
        static const char* topicPublishSensorMotionDetected; // sensor motion detected
        static const char* topicPublishSensorTemperature;    // sensor temperature data
        static const char* topicPublishSensorHumidity;       // sensor humidity data
        static const char* topicPublishSensorIaq;            // sensor IAQ data
        static const char* topicPublishSensorIaqAccuracy;    // sensor IAQ accuracy data
        static const char* topicPublishSensorCo2;            // sensor CO2 data
        static const char* topicPublishSensorGasResistance;  // sensor gas resistance data (Ohms)
        static const char* topicPublishSensorIlluminance;    // sensor illuminance data

        // MQTT subscription topics
        static const char* topicSubscribeEnableMqttDebug;
        static const char* topicSubscribeClientCommand;
};

#endif

