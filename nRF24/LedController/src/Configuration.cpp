#include "Configuration.h"

static uint8_t  ADDRESS_BYTE_DEFAULT = 'p';

Configuration::Configuration()
{
    // not initialized, set default values
    clientId = 255;             // default client ID 255
    timeout  = 60;              // default timeout 60 seconds
    pwmValue = 10;              // default PWM value 100%
    illuminanceThreshold = 20;  // default illuminance threshold 20 lux
    ledCount = 0;               // default LED count 0
    addressByte = ADDRESS_BYTE_DEFAULT;          // default address byte 'o'
    sleepPeriod = 3600;         // default sleep period 3600 seconds (1 hour)
    longClickSupported = 0;     // default: button long click not supported
    txPowerLevel = 2;           // default TX power level 2
}

void Configuration::init()
{
    // check if EEPRIM has been initialized with default values
    uint8_t isInitialized = readByteFromEEPROM(ADDRESS_IS_INITIALIZED);
    if (isInitialized != MAGIC_NUMBER) {
        // not initialized, set default values
        clientId = 255;             // default client ID 255
        timeout  = 0;               // default timeout 60 seconds
        pwmValue = 10;              // default PWM value 100%
        illuminanceThreshold = 20;  // default illuminance threshold 20 lux
        ledCount = 0;               // default LED count 0
        addressByte = ADDRESS_BYTE_DEFAULT;          // default address byte 'o'
        sleepPeriod = 3600;         // default sleep period 3600 seconds (1 hour)
        longClickSupported = 0;     // default: button long click not supported
        txPowerLevel = 2;           // default TX power level 2

        // write defaults to EEPROM
        writeByteToEEPROM(ADDRESS_IS_INITIALIZED, MAGIC_NUMBER);
        writeByteToEEPROM(ADDRESS_CLIENT_ID, clientId);
        writeWordToEEPROM(ADDRESS_TIMEOUT, timeout);
        writeByteToEEPROM(ADDRESS_PWM_VALUE, pwmValue);
        writeByteToEEPROM(ADDRESS_ILLUMINANCE, illuminanceThreshold);
        writeByteToEEPROM(ADDRESS_LED_COUNT, ledCount);
        writeByteToEEPROM(ADDRESS_ADDRESS_BYTE, addressByte);
        writeWordToEEPROM(ADDRESS_SLEEP_PERIOD, sleepPeriod);
        writeByteToEEPROM(ADDRESS_LONG_CLICK, longClickSupported);
        writeByteToEEPROM(ADDRESS_TX_POWER_LEVEL, txPowerLevel);
    } else {
        // read values from EEPROM
        clientId             = readByteFromEEPROM(ADDRESS_CLIENT_ID);
        timeout              = readWordFromEEPROM(ADDRESS_TIMEOUT);
        pwmValue             = readByteFromEEPROM(ADDRESS_PWM_VALUE);
        illuminanceThreshold = readByteFromEEPROM(ADDRESS_ILLUMINANCE);
        ledCount             = readByteFromEEPROM(ADDRESS_LED_COUNT);
        addressByte          = readByteFromEEPROM(ADDRESS_ADDRESS_BYTE);
        sleepPeriod          = readWordFromEEPROM(ADDRESS_SLEEP_PERIOD);
        longClickSupported   = readByteFromEEPROM(ADDRESS_LONG_CLICK);
        txPowerLevel         = readByteFromEEPROM(ADDRESS_TX_POWER_LEVEL);
        if(addressByte != 'e' && addressByte != 'o' && addressByte != 'u' && addressByte != 'p' ) {
          // invalid address byte, reset to default
          addressByte = ADDRESS_BYTE_DEFAULT;
          writeByteToEEPROM(ADDRESS_ADDRESS_BYTE, addressByte);
        }
    }

    // ensure valid RF TX power level
    if(txPowerLevel > 3) {
      txPowerLevel = 2; // reset to default
      writeByteToEEPROM(ADDRESS_TX_POWER_LEVEL, txPowerLevel);
    }
}


uint8_t Configuration::getClientId()
{
    return clientId;
}
void Configuration::setClientId(uint8_t clientId)
{
    this->clientId = clientId;
    writeByteToEEPROM(ADDRESS_CLIENT_ID, clientId);
}

uint16_t Configuration::getTimeout()
{
    return timeout;
}
void Configuration::setTimeout(uint16_t timeout)
{
    this->timeout = timeout;
    writeWordToEEPROM(ADDRESS_TIMEOUT, timeout);
}

uint8_t Configuration::getPwmValue()
{
    return pwmValue;
}
void Configuration::setPwmValue(uint8_t pwmValue)
{
    this->pwmValue = pwmValue;
    writeByteToEEPROM(ADDRESS_PWM_VALUE, pwmValue);
}

uint8_t Configuration::getIlluminanceThreshold()
{
    return illuminanceThreshold;
}
void Configuration::setIlluminanceThreshold(uint8_t threshold)
{
    this->illuminanceThreshold = threshold;
    writeByteToEEPROM(ADDRESS_ILLUMINANCE, threshold);
}

uint8_t Configuration::getLedCount()
{
    return ledCount;
}
void Configuration::setLedCount(uint8_t ledCount)
{
    this->ledCount = ledCount;
    writeByteToEEPROM(ADDRESS_LED_COUNT, ledCount);
}

uint8_t Configuration::getAddressByte()
{
    return addressByte;
}
void Configuration::setAddressByte(uint8_t addressByte)
{
    this->addressByte = addressByte;
    writeByteToEEPROM(ADDRESS_ADDRESS_BYTE, addressByte);
}

uint16_t Configuration::getSleepPeriod()
{
    return sleepPeriod;
}
void Configuration::setSleepPeriod(uint16_t sleepPeriod)
{
    this->sleepPeriod = sleepPeriod;
    writeWordToEEPROM(ADDRESS_SLEEP_PERIOD, sleepPeriod);
}

uint8_t Configuration::getLongClickSupported()
{
    return Configuration::longClickSupported;
}
void Configuration::setLongClickSupported(uint8_t longClickSupported)
{  
    this->longClickSupported = longClickSupported;
    writeByteToEEPROM(ADDRESS_LONG_CLICK, longClickSupported);
}   

uint8_t Configuration::getTxPowerLevel()
{
    return txPowerLevel;
}
void Configuration::setTxPowerLevel(uint8_t txPowerLevel)
{
    this->txPowerLevel = txPowerLevel;
    writeByteToEEPROM(ADDRESS_TX_POWER_LEVEL, txPowerLevel);
}


uint8_t  Configuration::readByteFromEEPROM(uint8_t address)
{
    /* Wait for completion of previous write */
    while(EECR & (1<<EEPE))
        ;
    
    /* Set up address register */
    EEARL = address;
    /* Start eeprom read by writing EERE */
    EECR |= (1<<EERE);

    /* Return data from data register */
    return EEDR;
}

void Configuration::writeByteToEEPROM(uint8_t address, uint8_t value)
{
    /* Wait for completion of previous write */
    while(EECR & (1<<EEPE))
        ;

    /* Set Programming mode */
    EECR = (0<<EEPM1)|(0>>EEPM0);
    /* Set up address and data registers */
    EEARL = address;
    EEDR  = value;
    /* Write logical one to EEMPE */
    EECR |= (1<<EEMPE);
    /* Start eeprom write by setting EEPE */
    EECR |= (1<<EEPE);
}

uint16_t Configuration::readWordFromEEPROM(uint8_t address)
{
    uint8_t lowByte  = readByteFromEEPROM(address);
    uint8_t highByte = readByteFromEEPROM(address + 1);
    return (highByte << 8) | lowByte;
}

void Configuration::writeWordToEEPROM(uint8_t address, uint16_t value)
{
    // Implement EEPROM write for a word
    writeByteToEEPROM(address, value & 0xFF);            // Write low byte
    writeByteToEEPROM(address + 1, (value >> 8) & 0xFF); // Write high byte
}
