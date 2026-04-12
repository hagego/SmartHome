/**
 * BatteryLight - Battery-powered light with motion sensor and nRF24L01+ wireless control
 * Uses ATtiny84, PIR motion sensor, and LED driver with PWM brightness control
 * 
 * ATtiny84 pinout:
 * - pin  2: PB0,PCINT8  - Motion sensor 1 input
 * - pin  3: PB1         - CE for nRF24L01
 * - pin  4: PB3,PCINT11 - Motion sensor 2 input - requires fusing PB3 as GPIO
 * - pin  5: PB2         - CS for nRF24L01
 * - pin  6: PA7,OC0B    - PWM output to LED driver or data pin for WS2812 strip 1
 * - pin  7: PA6,MOSI    - SPI MOSI for nRF24LO1
 * - pin  8: PA5,MISO    - SPI MISO for nRF24LO1
 * - pin  9: PA4,SCK     - SPI SCK for nRF24LO1
 * - pin 10: PA3         - DCDC enable for LED driver or data pin for WS2812 strip 2 or pull-up button input as motion sensor 3
 * - pin 11: PA2         - IIC SDA for BH1750FVI
 * - pin 12: PA1         - IIC SCL for BH1750FVI
 * - pin 13: PA0,ADC0    - Battery voltage measurement
 */

#include <Arduino.h>

#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/wdt.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#ifdef LED_TYPE_WS2812
  #include "light_ws2812.h"
#endif

#ifdef ILLUMINANCE_SENSOR
  #include "BH1750.h"
#endif

#ifdef ENV_SENSOR
  #include "BME280I2C.h"
#endif

#include "Configuration.h"


// wait time after sending data in microseconds
const uint16_t POST_SEND_DELAY_US = 30000; // 30ms

// threshold for long button press in ms
const uint16_t LONG_PRESS_THRESHOLD_MS = 700;

// resistive divider for battery voltage measurement
const double R_VCC = 470000.0;  // VCC to sense point 470k
const double R_GND = 100000.0;  // sense point to GND 100k
const double VREF  = 1.1;       // internal reference voltage

// nRF24 addresses to use (channel) (5 bytes)
// address 0 is used for writing, address 1 for reading
uint8_t nRF24Addresses[][6] = {"pclnt", "ctrl "}; 

const uint8_t nRF24PayloadSize = 16; // max. 32 bytes possible

// global configuration object
Configuration config;

// global RF24 object and payload buffer
RF24          radio(PB1, PB2);           // create a global RF24 object, CE, CSN
char          payload[nRF24PayloadSize]; // create a payload buffer
char          ack[1];                    // create an ack buffer

// global LED array for WS2812
#ifdef LED_TYPE_WS2812
  const uint8_t MAX_NUM_LEDS = 64;
  struct cRGB  ledArray[MAX_NUM_LEDS];
#endif

// Function prototypes
void   initAdc();                              // initialize ADC for battery voltage measurement
void   readAndSendBatteryVoltage();            // measures battery voltage and sends it via nRF24
void   reportConfiguration();                  // send current configuration via nRF24

#ifdef ENABLE_SLEEP
void   enterSleep();                           // enter sleep mode
#endif

#ifdef ILLUMINANCE_SENSOR
float  readIlluminance();               // read illuminance using BH1750 sensor and send via nRF24
#endif

#ifdef LED_TYPE_PWM
void   initializePWM();                        // initialize PWM on PA7
void   setPWMDutyCycle(uint8_t dutyCycle);     // set PWM duty cycle on PA7 (0-100)
#endif

#ifdef LED_TYPE_WS2812
void   initializeLedStrip();                      // initialize LED strip with all off
void   setLedStripPattern(uint8_t patternCode);   // set LED strip pattern based on pattern code
#endif

#ifdef ENV_SENSOR
void readAndSendEnvironmentalData();
#endif

#ifdef SERVO
void positionServo(uint16_t pulseWidth); // move servo to specified position
#endif


boolean justStarted = true;
void setup() {

  /**
 * pinout:
 * - pin  2: PB0,PCINT8  - Motion sensor 1 input
 * - pin  3: PB1         - CE for nRF24L01
 * - pin  4: PB3,PCINT11 - Motion sensor 2 input - requires fusing PB3 as GPIO
 * - pin  5: PB2         - CS for nRF24L01
 * - pin  6: PA7,OC0B    - PWM output to LED driver or data pin for WS2812 strip 1
 * - pin  7: PA6,MOSI    - SPI MOSI for nRF24LO1
 * - pin  8: PA5,MISO    - SPI MISO for nRF24LO1
 * - pin  9: PA4,SCK     - SPI SCK for nRF24LO1
 * - pin 10: PA3         - DCDC enable for LED driver or data pin for WS2812 strip 2 or pull-up button input as motion sensor 3
 * - pin 11: PA2         - IIC SDA for BH1750FVI
 * - pin 12: PA1         - IIC SCL for BH1750FVI
 * - pin 13: PA0,ADC0    - Battery voltage measurement
 */

  // ensure timer0 runs at 1MHz
  #if(F_CPU == 1000000L)
    TCCR0B = _BV(CS01);                  // Prescaler = 8
  #elif(F_CPU == 8000000L)
    TCCR0B = _BV(CS00) | _BV(CS01);      // Prescaler = 64
  #else
    #error "Unsupported F_CPU for PWM initialization"
  #endif

  // wait 1s before initializing configuration
  for(uint8_t i=0; i<20; i++)
  {
    delayMicroseconds(50000);
  } 
  // initialize configuration
  config.init();

  // set pin modes
  DDRB  &= ~_BV(PB0);            // pin  2: Set PB0 as input: motion sensor 1
  DDRB  |=  _BV(PB1);            // pin  3: Set PB1 as output: CE for nRF24L01
  DDRB  &= ~_BV(PB3);            // pin  4: Set PB3 as input: motion sensor 2
  DDRB  |=  _BV(PB2);            // pin  5: Set PB2 as output: CS for nRF24L01
  DDRA  |=  _BV(PA7);            // pin  6: Set PA7 as output for PWM to LED driver
  DDRA  |=  _BV(PA6);            // pin  7: Set PA6 as output: SPI MOSI for nRF24LO1
  DDRA  &= ~_BV(PA5);            // pin  8: Set PA5 as input: SPI MISO for nRF24LO1
  DDRA  |=  _BV(PA4);            // pin  9:Conn_01x03 Set PA4 as output: SPI SCK for nRF24LO1
  #if defined(LED_TYPE_PWM) || defined(LED_TYPE_WS2812) || defined(SERVO)
  DDRA  |=  _BV(PA3);            // pin 10: Set PA3 as output: enables DCDC for LED driver
  PORTA &= ~_BV(PA3);            // Set PA3 low: disables DCDC for LED driver => LED off
  #else
  DDRA  &= ~_BV(PA3);            // pin 10: Set PA3 as input: motion sensor 3
  PORTA |=  _BV(PA3);            // Enable pull-up resistor for motion sensor 3 pin
  #endif
  DDRA  |=  _BV(PA2);            // pin 11: Set PA2 as output: IIC SDA for BH1750FVI
  DDRA  |=  _BV(PA1);            // pin 12: Set PA1 as output: IIC SCL for BH1750FVI

  #ifdef LED_TYPE_PWM
    initializePWM();                     // initialize PWM on PA7
  #endif


  // store client ID in 1st byte of payload and ack buffer
  payload[0] = config.getClientId();
  ack[0]     = config.getClientId();
  payload[2] = ':';

  nRF24Addresses[0][0] = (uint8_t)config.getAddressByte();

  // initialize nRF24L01
  radio.begin();

  radio.stopListening();                     // set module as transmitter
  radio.setAutoAck(1);                       // Ensure autoACK is enabled
  radio.enableAckPayload();                  // enable payloads within ACK packets
  radio.setRetries(5,20);                    // Max delay between retries & number of retries
  radio.setPayloadSize(nRF24PayloadSize);    // Set payload size to 16 bytes
  radio.setPALevel((rf24_pa_dbm_e)config.getTxPowerLevel());      // Set power level
  radio.stopListening(nRF24Addresses[0]);    // switch to writing on pipe 0

  delayMicroseconds(50000);                  // wait 50ms to settle current after powering up radio

  // initialize ADC for battery voltage measurement
  initAdc();

  // send connect message
  payload[1] = 'C';
  payload[3] = '1';
  payload[4] = 0;
  radio.write( payload,sizeof(payload) );

  // report current configuration
  reportConfiguration();

  #ifdef ILLUMINANCE_SENSOR
    // measure illuminance
    float lux = readIlluminance();

    payload[1] = 'I';
    utoa((uint16_t)(lux), payload+3, 10);
    radio.write( payload,sizeof(payload) );  // Send data
  #endif

  // measure battery voltage and send
  readAndSendBatteryVoltage();

  // enable the WD interrupt (note no reset)
  // set up WDT for interrupt only mode every 8s
  wdt_reset();     // Reset the WDT timer
  cli();           // disable interrupts
  
  /* Clear WDRF in MCUSR */
  MCUSR &= ~(1 << WDRF); // Clear WDRF
  /* Write logical one to WDCE and WDE */
  WDTCSR |= (1<<WDCE) | (1<<WDE);
  /* Set new prescaler(time-out) value = 8s and enable */
  WDTCSR |= _BV(WDE) |_BV(WDIE) | _BV(WDP0) | _BV(WDP3);
  sei();

  #ifdef ENV_SENSOR
    // measure environmental data
  readAndSendEnvironmentalData();
  #endif


  #ifdef LED_TYPE_WS2812
    // send LED count
    payload[1] = 'N';
    utoa(config.getLedCount(), payload+3, 10);
    radio.write( payload,sizeof(payload) );
  #endif

  radio.txStandBy();                         // Wait for the transmission to complete
}

bool    ledStripPatternEnabled = false;
uint8_t wakeupSource = 0;                     // 0 = none, 1 = motion sensor 1, 2 = motion sensor 2, 3 = button input as motion sensor 3


//
// start LOOP
//
boolean gotoSleepAgain = false;
boolean pinChangeInterruptTriggered = false;
void loop() {
  #ifdef ENABLE_SLEEP
    if(!justStarted) {
      radio.powerDown();                         // Power down the radio
    }

    #ifdef LED_TYPE_PWM
      PORTA &= ~_BV(PA3);                        // Set PA3 low: disables DCDC for LED driver => LED off
    #endif

    // go to sleep until motion is detected
    // skips sleep on first run after power-up
    if(!justStarted) {

      gotoSleepAgain = true;
      pinChangeInterruptTriggered = false;
      // sleep until woken up by motion sensor or watchdog timer, if woken up by watchdog timer go back to sleep until motion is detected or timeout is reached
      while(gotoSleepAgain ) {
        enterSleep();
      }

      // waking up
    }

    // check and store wakeup source
    wakeupSource = 0; 

    #ifdef MOTION_SENSOR
      if((PINB & _BV(PB0)) == _BV(PB0)) {
        wakeupSource = 1;
        payload[3] = '1';
      }
      if((PINB & _BV(PB3)) == _BV(PB3)) {
        wakeupSource = 2;
        payload[3] = '2';
      }
    #endif

    #ifdef BUTTON
      // check if PA3 is low (as button input)
      if((PINA & _BV(PA3)) == 0) {
        wakeupSource = 3;
      
        if(config.getLongClickSupported()) {
          // wait until PA3 goes high again (button released)
          uint64_t buttonStartTime = millis();
          while((PINA & _BV(PA3)) == 0) {
            delayMicroseconds(1000);
          }
          uint64_t buttonPressDuration = millis() - buttonStartTime;
          if(buttonPressDuration > LONG_PRESS_THRESHOLD_MS) {
            // long press detected
            wakeupSource = 4;
          }
        }
      }
    #endif // BUTTON

    #ifdef MOTION_SENSOR
      if(pinChangeInterruptTriggered && wakeupSource == 0) {
        return; // motion sensor input pins changed back to low
      }
    #endif

    #ifdef ILLUMINANCE_SENSOR
      // measure illuminance
      float lux = readIlluminance();

      if(wakeupSource == 1 || wakeupSource==2) {
        if(config.getIlluminanceThreshold()>0 && lux >= config.getIlluminanceThreshold()) {
          // ambient light is sufficient go back to sleep
          return;
        }
        else {
          #ifdef LED_TYPE_PWM
            // ambient light is insufficient, turn on LED
            setPWMDutyCycle(config.getPwmValue());         // set PWM to configured brightness
            PORTA |= _BV(PA3);                             // Set PA3 high: enables DCDC for LED driver
          #endif
        }
      }
    #endif

        // power up radio
    radio.powerUp();
    delayMicroseconds(POST_SEND_DELAY_US);     // wait for radio to stabilize
    radio.setRetries(5,20);                    // Max delay between retries & number of retries
    radio.setPALevel((rf24_pa_dbm_e)config.getTxPowerLevel());    // Set power level
    radio.stopListening(nRF24Addresses[0]);    // switch to writing on pipe 0 
    delayMicroseconds(POST_SEND_DELAY_US);     // wait to settle current after switching to transmitter mode

    #ifdef ILLUMINANCE_SENSOR
      payload[1] = 'I';
      utoa((uint16_t)(lux), payload+3, 10);
      radio.write( payload,sizeof(payload) );  // Send data
    #endif

    // send motion detected message
    if(wakeupSource!=0) {
      #if defined(MOTION_SENSOR) || defined(BUTTON)
        payload[1] = 'M';
        payload[3] = '0'+wakeupSource;
        payload[4] = 0;

        radio.write( payload,sizeof(payload) );
        delayMicroseconds(POST_SEND_DELAY_US);
      #endif
    }
    else {
      // periodic wakeup. Measure battery voltage and send
      readAndSendBatteryVoltage();
      delayMicroseconds(POST_SEND_DELAY_US);
    }

    #ifdef ENV_SENSOR
      // measure environmental data
      readAndSendEnvironmentalData();
    #endif

  #endif // ENABLE_SLEEP

  justStarted = false;

  // send ready to listen message
  payload[1] = 'L';
  payload[3] = '1';
  payload[4] = 0;
  radio.write( payload,sizeof(payload) );


  delayMicroseconds(POST_SEND_DELAY_US);
  radio.txStandBy();              // Wait for the transmission to complete
  
  #ifdef BUTTON
    //ensure PA3 goes high again (button released)
    while((PINA & _BV(PA3)) == 0) {
      delayMicroseconds(1000);
    }
  #endif
  // Now set the module as receiver and wait for commands
  radio.openReadingPipe(1, nRF24Addresses[1]);
  radio.startListening();

  boolean exitCondition = false;

  uint64_t startTime = millis();
  radio.writeAckPayload(0, ack, sizeof(ack)); // send ack payload with client ID
  while (    !exitCondition
          && (   (config.getTimeout() == 0)
              || ((millis() - startTime) < (uint64_t)config.getTimeout() * 1000UL))) {
    uint8_t pipe;
    if(radio.available(&pipe)) {
      radio.writeAckPayload(pipe, ack, sizeof(ack)); // send ack payload with client ID
      char text[nRF24PayloadSize] = {0};
      radio.read(&text, sizeof(text));

      // get target client ID from message
      uint8_t targetClientId = text[0];

      // client ID 255 is broadcast
      if(targetClientId != config.getClientId() && targetClientId != 255) {
        // not for me
        continue;
      }

      // process commands

      // command to trigger a report of current configuration: "R"
      if(text[1]=='R') {
        radio.stopListening();          // set module as transmitter
        delayMicroseconds(50000);       // wait 50ms to settle current after powering up radio

        reportConfiguration();
        radio.txStandBy();

        // return to listening mode
        radio.openReadingPipe(1, nRF24Addresses[1]);
        radio.startListening();   
      }

      if(strlen(text+1)>2) {
        // command to set nRF24 send pipe address byte: "A:<value>" where <value> is a single character
        if(text[1]=='A') {
          config.setAddressByte((uint8_t)text[3]);
          nRF24Addresses[0][0] = (uint8_t)config.getAddressByte();
        }

        // ensure command has a valid parameter
        uint16_t parameter = (uint16_t)atol(text + 3);
        if(parameter>=0) {
          // command to set new client ID: "X:<value>" where <value> is 0-255
          if(text[1]=='X' && (targetClientId==config.getClientId() || config.getClientId()==255)) {
            uint8_t clientId = (uint8_t)parameter;
            if(clientId > 0 && clientId <= 255) {
              // set new client ID and update payload
              config.setClientId((uint8_t)clientId);
              payload[0] = config.getClientId();
            }
          }
          
          // command to set timeout value: "T:<value>" where <value> is in seconds
          if(text[1]=='T' ) {
            config.setTimeout((uint16_t)parameter);
          }

          // command to set sleep period value: "S:<value>" where <value> is in seconds
          if(text[1]=='S' ) {
            config.setSleepPeriod((uint16_t)parameter);
          }

          // command to set nRF24 TX power level: "W:<value>" where <value> is 0-3
          if(text[1]=='W') {
            uint8_t txPowerLevel = (uint8_t)parameter;
            if(txPowerLevel <= 3) {
              config.setTxPowerLevel(txPowerLevel);
              radio.setPALevel((rf24_pa_dbm_e)config.getTxPowerLevel());
            }
          }

          #ifdef BUTTON
            // command to set if long button click is supported: "L:<value>" where <value> is 0 or 1
            if(text[1]=='B' ) {
              uint8_t longClickSupported = (uint8_t)parameter;
              if(longClickSupported == 0 || longClickSupported == 1) {
                config.setLongClickSupported(longClickSupported);
              }
            }
          #endif

          #ifdef LED_TYPE_PWM
            // command to set PWM value: "P:<value>" where <value> is 0-100
            if(text[1]=='P') {
              int pwmValue = parameter;
              if(pwmValue <= 100) {
                config.setPwmValue((uint8_t)pwmValue);
                setPWMDutyCycle((uint8_t)pwmValue);
              }
            }
          #endif

          #ifdef SERVO
          // servo position from command: "P:<value>" where <value> is pulse width in microseconds
            if(text[1]=='P') {
              radio.stopListening();          // set module as transmitter
              payload[1] = 'P';
              payload[2] = ':';

              positionServo(parameter); // move servo to specified position

              // send end position back as confirmation
              utoa((uint16_t)parameter, payload+3, 10);
              radio.write( payload,sizeof(payload) );
              delayMicroseconds(POST_SEND_DELAY_US);

              radio.txStandBy();

              // return to listening mode
              radio.openReadingPipe(1, nRF24Addresses[1]);
              radio.startListening(); 
            }
          #endif

          #ifdef ILLUMINANCE_SENSOR
            // command to set illuminance threshold: "I:<value>" where <value> is in lux
            if(text[1]=='I') {
              int threshold = parameter;
              if(threshold <= 255) {
                config.setIlluminanceThreshold((uint8_t)threshold);
              }
            }
          #endif

          #ifdef LED_TYPE_WS2812
            // command to apply a pattern to the LED strip: "L:<pattern>" where <pattern> is pattern code
            if(text[1]=='L') {
              uint8_t patternCode = (uint8_t)parameter;
              setLedStripPattern(patternCode);
            }

            // command to set LED count: "N:<value>" where <value> is number of LEDs
            if(text[1]=='N' ) {
              if(parameter <= MAX_NUM_LEDS) {
                config.setLedCount((uint8_t)parameter);
              }
            }
          #endif
        }
      }


    } // if(radio.available(&pipe))


    #ifdef BUTTON
      // check for pull-up button input as motion sensor 3
      if((PINA & _BV(PA3)) == 0) {
        radio.stopListening();          // set module as transmitter

        payload[1] = 'M';
        payload[3] = '3';
        payload[4] = 0;

        if(!config.getLongClickSupported()) {
          // send immediately that button was pressed
          radio.write( payload,sizeof(payload) );
        }

        // wait until PA3 goes high again (button released)
        uint64_t buttonStartTime = millis();
        while((PINA & _BV(PA3)) == 0) {
          delayMicroseconds(1000);
        }
        uint64_t buttonPressDuration = millis() - buttonStartTime;
        if(buttonPressDuration > LONG_PRESS_THRESHOLD_MS) {
          // long press detected, set wakeup source to 0 to ignore motion sensor 3 for now
          payload[3] = 'L';
        }

        if(config.getLongClickSupported()) {
          radio.write( payload,sizeof(payload) );
        }

        radio.txStandBy();
        radio.openReadingPipe(1, nRF24Addresses[1]);
        radio.startListening();   

        // wait 400ms for debounce
        for(uint16_t i=0; i<400; i++) {
          delayMicroseconds(1000);
        }
      }

    #endif // BUTTON

    #if defined(LED_TYPE_PWM) && defined(MOTION_SENSOR)
      // stay awake until motion detection is cleared
      if(wakeupSource == 1 && ((PINB & _BV(PB0)) == _BV(PB0))) {
        startTime = millis();
      } 
      if(wakeupSource == 2 && ((PINB & _BV(PB3)) == _BV(PB3))) {
        startTime = millis();
      }
    #endif

    delayMicroseconds(1000);
  } // while loop

  #ifdef MOTION_SENSOR
    // ensure motion sensor input pins are low again before going back to sleep to avoid immediate wakeup
    if(wakeupSource == 1) {
      while((PINB & _BV(PB0)) == 1) {
        delayMicroseconds(1000);
      }
    }
    if(wakeupSource == 2) {
      while((PINB & _BV(PB3)) == 1) {
        delayMicroseconds(1000);
      }
    }
  #endif

  radio.stopListening();          // set module as transmitter

  payload[1] = 'L';
  payload[3] = '0';
  payload[4] = 0;
  radio.write( payload,sizeof(payload) );
  delayMicroseconds(POST_SEND_DELAY_US);

  radio.txStandBy();              // Wait for the transmission to complete
}


/**
 * enter MCU sleep mode. nRF24 module is *not* handled in this function
 */
void enterSleep() {
  GIMSK  |= _BV(PCIE1);                  // Enable Pin Change Interrupts
  GIMSK  |= _BV(PCIE0);                  // Enable Pin Change Interrupts
 
  #if defined(MOTION_SENSOR) || defined(BUTTON)
  PCMSK0 |= _BV(PCINT3);                 // Enable pin change interrupt on PA3
  PCMSK1 |= _BV(PCINT8) | _BV(PCINT11);  // Enable pin change interrupt on PB0 and PB3
  #endif

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // replaces above statement
  sleep_enable();                        // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
  sei();                                 // enable interrupts
  sleep_cpu();                           // sleep
  // The CPU is now sleeping, and will wake up on interrupt
  // The ISR will be called when the interrupt occurs
  // The CPU will wake up and continue executing from here after ISR
  //cli();                               // Disable interrupts
  GIMSK  &= ~_BV(PCIE1);                 // Disable Pin Change Interrupts
  GIMSK  &= ~_BV(PCIE0);                 // Disable Pin Change Interrupts

  PCMSK0 &= ~_BV(PCINT3); 
  PCMSK1 &= ~_BV(PCINT8); 
  PCMSK1 &= ~_BV(PCINT11); 

  sleep_disable();                       // Disable sleep mode
}

#if defined(MOTION_SENSOR) || defined(BUTTON)
// ISR for motion sensor (pin change)
ISR (PCINT0_vect) {  
  gotoSleepAgain = false;
  pinChangeInterruptTriggered = true;
}
ISR (PCINT1_vect) {  
  gotoSleepAgain = false;
  pinChangeInterruptTriggered = true;
}
#endif

// ISR for watchdog timer
uint16_t wdtIsrCount = 0;
ISR (WDT_vect) {
  WDTCSR |= _BV(WDIE); // re-enable WDT interrupt

  wdtIsrCount++;
  if(wdtIsrCount>config.getSleepPeriod()/8) {
    wdtIsrCount = 0;
    gotoSleepAgain = false;
  }
  else {
    gotoSleepAgain = true;
  }
}

// ISR for bad interrupt
// This is a catch-all for any interrupts that don't have a specific handler
ISR(BADISR_vect) {
}


/**
 * initialize ADC
*/
void initAdc() {
  PRR    &= ~_BV(PRADC);      // make sure PRADC is clear
	
  // Disable Comparator
	ACSR &= ~_BV(ACIE);        // disable Analog Comparator interrupt
  ACSR |=  _BV(ACD);         // disable Comparator
	
	/*  ADCSRA Scaler
	//	ADPS2	ADPS1	ADPS0	Division Factor
	//	0		0		0		2
	//	0		0		1		2   (!)
	//	0		1		0		4
	//	0		1		1		8
	//	1		0		0		16
	//	1		0		1		32
	//	1		1		0		64	<--- needs to end up between 50kHz and 200kHz so 125kHz is OK
	//	1		1		1		128 <--- 65.5kHz also OK
	*/
  // ADCSRA selection - make sure ADEN is clear before changing ADPS[2:0]
  ADCSRA &= ~((1<<ADEN)|(1<<ADATE)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));  // mask out bits
  ADCSRA |=   (0<<ADEN)|(0<<ADATE)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(0<<ADPS0);   // set no auto trigger, interrupts enabled, ADC prescaler of 64
  
  // ADMUX selection - make sure ADEN is clear before changing MUX[5:0]
  ADMUX = 0b00000000;                 // clear all bits
	ADMUX |= _BV(REFS1);	              // use internal 1.1V ref, set ADC0 (PA0)

	ADCSRB &= ~((1<<BIN)|(1<<ADLAR));   // right justified  - 10 bit
  DIDR0  |=  _BV(ADC0D);              // disable digital input on ADC0

  ADCSRA &= ~_BV(ADEN);               // disable ADC again
}


/**
 * measure battery voltage on ADC and transmit it via nRF24
 */
void readAndSendBatteryVoltage(){
  power_adc_enable();                     // enable ADC
  ADCSRA |= _BV(ADEN);									  // enable ADC clock
	delayMicroseconds(10000);	              // settle
  ADCSRA |= _BV(ADSC);                    // start a conversion
  while (ADCSRA & (1<<ADSC)){}
  uint16_t reading = (ADCL | (ADCH<<8));	// it's important to read ADCL before ADCH
	ADCSRA &= ~_BV(ADEN);									  // disable ADC clock again
  power_adc_disable();                    // disable ADC to save power

  double voltage = (double)reading * VREF * (R_VCC + R_GND) / (R_GND * 1024.0); // calculate voltage

  payload[1] = 'V';
  // footprint of dtostrf is too large for ATtiny84. Use utoa and mV instead
  utoa((uint16_t)(voltage * 1000.0), payload+3, 10);
  radio.write( payload,sizeof(payload) );  // Send data
}


#ifdef LED_TYPE_WS2812

void initializeLedStrip() {
  uint8_t numLeds = config.getLedCount();

  for(uint8_t i=0; i<numLeds; i++) {
    ledArray[i].r = 0;
    ledArray[i].g = 0;
    ledArray[i].b = 0;
  }

  #ifdef ENABLE_SLEEP
    ws2812_setleds_pin(ledArray, numLeds, _BV(PA7) );           // use only PA7 for data, PA3 is used to power on the DCDC
  #else
    ws2812_setleds_pin(ledArray, numLeds, _BV(PA7) | _BV(PA3)); // use PA7 and PA3 for data
  #endif
}

void setLedStripPattern(uint8_t patternCode) {
  uint8_t numLeds = config.getLedCount();

  if(patternCode==0) {
    for(uint8_t i=0; i<numLeds; i++) {
      ledArray[i].r = 0;
      ledArray[i].g = 0;
      ledArray[i].b = 0;

      switch(i % 3) {
        case 0:
          ledArray[i].r = 255;
          break;
        case 1:
          ledArray[i].g = 255;
          break;
        case 2:
          ledArray[i].b = 255;
          break;
      }
    }
  }
  else {
    cRGB last = ledArray[numLeds-1];
    for(uint8_t i=numLeds-1; i>0; i--) {
      ledArray[i] = ledArray[i-1];
    }
    ledArray[0] = last;
  }

  #ifdef ENABLE_SLEEP
    ws2812_setleds_pin(ledArray, numLeds, _BV(PA7) );           // use only PA7 for data, PA3 is used to power on the DCDC
  #else
    ws2812_setleds_pin(ledArray, numLeds, _BV(PA7) | _BV(PA3)); // use PA7 and PA3 for data
  #endif
}

#endif

#ifdef LED_TYPE_PWM
/**
 * Initialize PWM for PA7 (ATtiny84 pin 6), connected to OC0B (Output Compare B for 8-bit Timer0)
 */
void initializePWM() {
  DDRA |= _BV(PA7);                      // Set PA7 as output

  // configure timer control
  TCCR0A = _BV(WGM00)  | _BV(WGM01) |    // Fast PWM mode with TOP = OxFF
           _BV(COM0B0) |_BV(COM0B1);     // set OC0B on compare match, clear at BOTTOM (inverted PWM)


           
  OCR0B = 0;                             // Initial duty cycle = 0%
}

/**
 * Set PWM duty cycle on PA7
 * @param dutyCycle Value from 0 to 100 representing duty cycle percentage
 */
void setPWMDutyCycle(uint8_t dutyCycle) {
  //if (dutyCycle > 100) dutyCycle = 100;
  
  // Calculate the compare value based on duty cycle
  OCR0B = (uint8_t)(((double)dutyCycle/100.0)*255.0);;
}
#endif


#ifdef ILLUMINANCE_SENSOR
/**
 * read illuminance using BH1750 sensor via direct I2C communication
 * @return illuminance in lux
 */
float readIlluminance() {
  float lux = 0.0;
  
  BH1750 lightMeter(0x23);            // Address 0x23 is the default address of the sensor
  lightMeter.write(BH1750_POWER_ON);  // Power on the sensor
  delayMicroseconds(10000);           // ensure enough time for Serial output in case of error

  // increase measurement time for better low-light accuracy
  if (lightMeter.configure(BH1750::ONE_TIME_HIGH_RES_MODE) && lightMeter.setMTreg(100)) {
    delayMicroseconds(50000U); // ensure enough time for measurement
    delayMicroseconds(50000U); // ensure enough time for measurement
    delayMicroseconds(50000U); // ensure enough time for measurement
    delayMicroseconds(50000U); // ensure enough time for measurement
    lux = lightMeter.readLightLevel();

    lightMeter.write(BH1750_POWER_DOWN);  // Power down the sensor
  }

  return lux;
}
#endif

#ifdef ENV_SENSOR
/**
 * read environmental data using BME280 sensor
 * Temperature is sent in degree Celsius multiplied by 10
 */
void readAndSendEnvironmentalData() {
  BME280I2C bme;

  if(bme.begin()) {
    payload[1] = 'D';
    itoa((int16_t)(bme.temp() * 10), payload+3, 10); 
    radio.write( payload,sizeof(payload) );  // Send data
    delayMicroseconds(POST_SEND_DELAY_US);
  }
}
#endif

void reportConfiguration() {
  // send client ID
  payload[1] = 'X';
  utoa((uint16_t)(config.getClientId()), payload+3, 10);
  radio.write( payload,sizeof(payload) );
  delayMicroseconds(POST_SEND_DELAY_US);

  // send address byte
  payload[1] = 'A';
  payload[3] = (char)(config.getAddressByte());
  payload[4] = 0;
  radio.write( payload,sizeof(payload) );
  delayMicroseconds(POST_SEND_DELAY_US);

  // send timeout setting
  payload[1] = 'T';
  utoa(config.getTimeout(), payload+3, 10);
  radio.write( payload,sizeof(payload) );
  delayMicroseconds(POST_SEND_DELAY_US);

  // send sleep period setting
  payload[1] = 'S';
  ultoa(config.getSleepPeriod(), payload+3, 10);
  radio.write( payload,sizeof(payload) );
  delayMicroseconds(POST_SEND_DELAY_US);

  // send TX power level
  payload[1] = 'W';
  payload[3] = '0' + config.getTxPowerLevel();
  payload[4] = 0;
  radio.write( payload,sizeof(payload) );
  delayMicroseconds(POST_SEND_DELAY_US);

  // send long click supported setting
  payload[1] = 'B';
  payload[3] = config.getLongClickSupported() ? '1' : '0';
  payload[4] = 0;
  radio.write( payload,sizeof(payload) );
  delayMicroseconds(POST_SEND_DELAY_US);

  #ifdef LED_TYPE_PWM
    // send PWM value
    payload[1] = 'P';
    utoa((uint16_t)(config.getPwmValue()), payload+3, 10);
    radio.write( payload,sizeof(payload) );
    delayMicroseconds(POST_SEND_DELAY_US);
  #endif

  #ifdef LED_TYPE_WS2812
    // send LED count
    payload[1] = 'N';
    utoa(config.getLedCount(), payload+3, 10);
    radio.write( payload,sizeof(payload) );
    delayMicroseconds(POST_SEND_DELAY_US);
  #endif

  #ifdef ILLUMINANCE_SENSOR
    // send illuminance threshold
    payload[1] = 'I';
    utoa((uint16_t)(config.getIlluminanceThreshold()), payload+3, 10);
    radio.write( payload,sizeof(payload) );
    delayMicroseconds(POST_SEND_DELAY_US);
  #endif
} 

#ifdef SERVO
void positionServo(uint16_t pulseWidth) {
  DDRA  &= ~_BV(PA7);       // Set PA7 as input to avoid unintended servo movement during power-up
  PORTA |= _BV(PA3);        // Set PA3 high: power on servo

  for(uint16_t i=0; i<10; i++) {
    delayMicroseconds(50000); // wait for C to charge and servo to power up
  }

  DDRA |= (1<<PA7);         // Set PA7 as output for PWM to servo

  uint16_t period     = 20000UL; // 20ms period for standard servos
  

  // hold end position for 4s to ensure servo reaches it
  for(uint16_t i=0; i<250; i++) {
    PORTA |= (1<<PA7);
    delayMicroseconds( pulseWidth );
    PORTA &= ~(1<<PA7);
    delayMicroseconds( period-pulseWidth );
  }

  // switch off power to servo after positioning to save power and avoid jitter
   PORTA &= ~_BV(PA3);  
 }
#endif