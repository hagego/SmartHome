#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <cstdarg>

#include "Debug.h"
#include "MqttInfo.h"

extern PubSubClient mqttClient;

bool Debug::mqttDebugEnabled = false;

void Debug::log(const char* format, ...) {
    va_list args;
    va_start(args, format);
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial.println(buffer);

    if (mqttDebugEnabled && mqttClient.connected()) {
        mqttClient.publish(MqttInfo::topicPublishDebugMessage, buffer);
    }
}

