/**
 * contains code to deal with servo commands
*/

#ifndef SERVO_H
#define SERVO_H

#include <Servo.h>
#include <EEPROM.h>

#include "EepromInfo.h"

extern char buffer[256];
extern PubSubClient mqttClient;

// process a servo command
void processServoCmd(uint8_t pinServoCtrl, char* cmd) {
  Servo servo;

  byte angleOpen  = EEPROM.read(EEPROM_ANGLE_OPEN);
  byte angleClose = EEPROM.read(EEPROM_ANGLE_CLOSE);

  byte newAngle = 0;
  byte oldAngle = 0;
  if(strcmp(cmd,"open")==0) {
    Serial.println(F("servo command open"));
    newAngle = angleOpen;
    oldAngle = angleClose;
  }

  if(strcmp(cmd,"close")==0) {
    Serial.println(F("servo command close"));
    newAngle = angleClose;
    oldAngle = angleOpen;
  }

  servo.attach(pinServoCtrl);
  servo.write(oldAngle);
  delay(200); // wait for servo to reach old position

  sprintf(buffer,"servo command is %s, moving from %d to %d",cmd,oldAngle,newAngle);
  mqttClient.publish(topicPublishDebug, buffer);

  if(oldAngle != newAngle) {
    if(oldAngle>newAngle) {
      for(int angle=oldAngle ; angle>newAngle ; angle-=10) {
        servo.write(angle);
        mqttClient.publish(topicPublishDebug, itoa(angle, buffer, 10));
        delay(200);
      }
    } else {
      for(int angle=oldAngle ; angle<newAngle ; angle+=10) {
        servo.write(angle);
        mqttClient.publish(topicPublishDebug, itoa(angle, buffer, 10));
        delay(200);
      }
    }
    servo.write(newAngle);
    delay(200);

    mqttClient.publish(topicPublishDebug, "servo done");
  }
}

#endif // SERVO_H