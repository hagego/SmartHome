/**
 * this file contains the definition of MQTT parameters
 */
#ifndef MQTT_INFO_H
#define MQTT_INFO_H

#define MQTT_SERVER "192.168.178.27"    // IP address of the MQTT broker
#define MQTT_PORT   1883                // port of the MQTT broker

class MqttInfo {
    public:
        // MQTT publish topics
        static const char* topicPublishConnected;
        static const char* topicPublishIsAlive;
        static const char* topicPublishDebugMessage;
        static const char* topicPublishClientMessage;

        // MQTT subscription topics
        static const char* topicSubscribeEnableMqttDebug;
        static const char* topicSubscribeClientCommand;
};

#endif
