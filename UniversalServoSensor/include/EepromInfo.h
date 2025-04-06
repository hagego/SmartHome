/**
 * contains EEPROM definitions
 */

#ifndef EEPROM_INFO_H
#define EEPROM_INFO_H

#include <EEPROM.h>

// EEPROM addresses
const uint8_t EEPROM_SIZE                = (uint8_t)64;   // size in bytes

const uint8_t EEPROM_SLEEP_TIME_MINUTES = 0;     // sleep time in minutes
const uint8_t EEPROM_ANGLE_OPEN         = 1;     // angle for open position
const uint8_t EEPROM_ANGLE_CLOSE        = 2;     // angle for close position

#endif