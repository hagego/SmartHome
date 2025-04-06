/**
 * contains code to deal with servo commands
*/

#ifndef SERVO_H
#define SERVO_H

#include <Servo.h>

// process a servo command
void processServoCmd(const char* cmd) {
  if(strcmp(cmd,"open")==0) {
    Serial.println(F("servo command open"));
  }
  if(strcmp(cmd,"close")==0) {
    Serial.println(F("servo command close"));
  }
}

#endif // SERVO_H