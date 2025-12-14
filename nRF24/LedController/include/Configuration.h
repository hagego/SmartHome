#include <Arduino.h>

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

/**
 * A simple class managing configuration data stored in the ATTiny EEPROM
 */
class Configuration {
  public:

    // default constructor
    Configuration();

    // initializes the configuration by reading from EEPROM or setting defaults
    void init();

    // gets/sets client ID
    uint8_t getClientId();
    void setClientId(uint8_t clientId);

    // gets/sets light timeout in seconds
    uint16_t getTimeout();
    void setTimeout(uint16_t timeout);

    // gets/sets PWM value in percent
    uint8_t getPwmValue();
    void setPwmValue(uint8_t pwmValue);

    // gets/sets illuminance threeshold in lux
    uint8_t getIlluminanceThreshold();
    void setIlluminanceThreshold(uint8_t threshold);

    // gets/sets WS2812 LED count
    uint8_t getLedCount();
    void setLedCount(uint8_t ledCount);

    // gets/sets nRF24 send pipe address byte
    uint8_t getAddressByte();
    void    setAddressByte(uint8_t addressByte);

    // gets/sets sleep period in seconds
    uint16_t getSleepPeriod();
    void     setSleepPeriod(uint16_t sleepPeriod);

  private:
  
    // data members
    uint8_t  clientId;
    uint16_t timeout;
    uint8_t  pwmValue;
    uint8_t  illuminanceThreshold;
    uint8_t  ledCount;
    uint8_t  addressByte;
    uint16_t sleepPeriod;

    // addresses
    static const uint8_t ADDRESS_IS_INITIALIZED = 0;  // stores a magic number if initialized
    static const uint8_t ADDRESS_CLIENT_ID      = 1;  // client ID (1 byte)
    static const uint8_t ADDRESS_TIMEOUT        = 2;  // timeout in seconds, 2 byte
    static const uint8_t ADDRESS_PWM_VALUE      = 4;  // PWM value in percent (1 byte)
    static const uint8_t ADDRESS_ILLUMINANCE    = 5;  // illuminance threshold in lux (1 byte)
    static const uint8_t ADDRESS_LED_COUNT      = 6;  // WS2812 LED count (1 byte)
    static const uint8_t ADDRESS_ADDRESS_BYTE   = 7;  // 1st byte of nRF24 send pipe (pipe 0) (1 byte)
    static const uint8_t ADDRESS_SLEEP_PERIOD   = 8;  // sleep period in seconds (2 byte)

    // magic number to check if EEPROM has been initialized with default data
    static const uint8_t MAGIC_NUMBER           = 42;

    // helper methods to read/write from/to EEPROM
    uint8_t  readByteFromEEPROM(uint8_t address);
    void     writeByteToEEPROM(uint8_t address, uint8_t value);
    uint16_t readWordFromEEPROM(uint8_t address);
    void     writeWordToEEPROM(uint8_t address, uint16_t value);
};

#endif // CONFIGURATION_H
