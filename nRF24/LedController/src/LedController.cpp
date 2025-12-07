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

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#ifdef LED_TYPE_WS2812
  #include "light_ws2812.h"
#endif

#ifdef NEEDS_WAKEUP
  #include "BH1750.h"
#endif

#include "Configuration.h"



// resistive divider for battery voltage measurement
const double R_VCC = 470000.0;  // VCC to sense point 470k
const double R_GND = 100000.0;  // sense point to GND 100k
const double VREF  = 1.1;       // internal reference voltage

// nRF24 addresses to use (channel) (5 bytes)
// address 0 is used for writing, address 1 for reading
uint8_t nRF24Addresses[][6] = {RF24_ADDR_SEND, RF24_ADDR_RECEIVE}; 

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

#ifdef NEEDS_WAKEUP
void   enterSleep();                           // enter sleep mode
float  readAndSendIlluminance();               // read illuminance using BH1750 sensor and send via nRF24
#endif

#ifdef LED_TYPE_PWM
void   initializePWM();                        // initialize PWM on PA7
void   setPWMDutyCycle(uint8_t dutyCycle);     // set PWM duty cycle on PA7 (0-100)
#endif

#ifdef LED_TYPE_WS2812
void   initializeLedStripPattern(bool enablePattern);             // initialize LED strip pattern
void   stepLedStripPattern();                   // step thru LED strip pattern
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

  // wait 500ms before initializing configuration
  for(uint8_t i=0; i<10; i++)
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
  DDRA  |=  _BV(PA4);            // pin  9: Set PA4 as output: SPI SCK for nRF24LO1
  #if defined(LED_TYPE_PWM) || defined(LED_TYPE_WS2812)
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

  #ifdef LED_TYPE_WS2812
    initializeLedStripPattern(false);
  #endif

  #ifdef LED_PATTERN_DEBUG
    config.setLedCount(10); // use 3 LEDs for pattern debug
    initializeLedStripPattern(false);

    while(true) {
      stepLedStripPattern();
      
      for(uint16_t i=0; i<1000; i++) {
        delayMicroseconds(1000);
      } 
    }
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
  radio.setRetries(5,15);                    // Max delay between retries & number of retries
  radio.setPayloadSize(nRF24PayloadSize);    // Set payload size to 16 bytes
  radio.setPALevel(RF24_PA_HIGH);            // Set power level to high
  radio.stopListening(nRF24Addresses[0]);    // switch to writing on pipe 0

  // initialize ADC for battery voltage measurement
  initAdc();

  // send connect message
  payload[1] = 'C';
  payload[3] = '1';
  payload[4] = 0;
  radio.write( payload,sizeof(payload) );

  // send timeout
  payload[1] = 'T';
  itoa(config.getTimeout(), payload+3, 10);
  radio.write( payload,sizeof(payload) );

  // measure battery voltage and send
  readAndSendBatteryVoltage();

  // measure illuminance and send
  #ifdef LED_TYPE_PWM
    readAndSendIlluminance();
  #endif

  #ifdef LED_TYPE_WS2812
    // send LED count
    payload[1] = 'N';
    utoa(config.getLedCount(), payload+3, 10);
    radio.write( payload,sizeof(payload) );
  #endif

  radio.txStandBy();                         // Wait for the transmission to complete
}

bool ledStripPatternEnabled = false;
void loop() {
  #ifdef NEEDS_WAKEUP
    radio.powerDown();                         // Power down the radio immediately after sending

    #ifdef LED_TYPE_PWM
      PORTA &= ~_BV(PA3);                        // Set PA3 low: disables DCDC for LED driver => LED off
    #endif

    // go to sleep until motion is detected
    // skips sleep on first run after power-up
    if(!justStarted) {
      enterSleep();
    }

    // power up radio
    radio.powerUp();
    radio.stopListening(nRF24Addresses[0]);  // switch to writing on pipe 0

    // measure illuminance
    float illuminance = readAndSendIlluminance();

    #ifdef LED_TYPE_PWM
      if(illuminance <= config.getIlluminanceThreshold()) {
        // ambient light is insufficient, turn on LED
        setPWMDutyCycle(config.getPwmValue());         // set PWM to configured brightness
        PORTA |= _BV(PA3);                             // Set PA3 high: enables DCDC for LED driver
      }
    #endif

    // send motion detected message
    payload[1] = 'M';
    payload[3] = '0';
    payload[4] = 0;

    if((PINB & _BV(PB0)) == _BV(PB0)) {
      payload[3] = '1';
    }
    if((PINB & _BV(PB3)) == _BV(PB3)) {
      payload[3] = '2';
    }
    // check if PA3 is low
    if((PINA & _BV(PA3)) == 0) {
      payload[3] = '3';
    }

    radio.write( payload,sizeof(payload) );

    // measure battery voltage and send
    readAndSendBatteryVoltage();   // read voltage

    // wait 500ms for debounce
    for(uint16_t i=0; i<500; i++) {
      delayMicroseconds(1000);
    }
  #endif
  justStarted = false;
  

  // send ready to listen message
  payload[1] = 'L';
  payload[3] = '1';
  payload[4] = 0;
  radio.write( payload,sizeof(payload) );

  radio.txStandBy();              // Wait for the transmission to complete
  delayMicroseconds(10000);

  // Now set the module as receiver and wait for commands
  radio.openReadingPipe(1, nRF24Addresses[1]);
  radio.startListening();

  boolean exitCondition = false;

  uint64_t startTime = millis();
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

      // command to switch off: "O"
      if(text[1]=='O') {
        exitCondition = true;
      }

      if(strlen(text+1)>2) {
        // ensure command has a valid parameter
        int parameter = atoi(text + 3);
        if(parameter>=0) {
          // command to set new client ID: "X:<value>" where <value> is 0-255
          if(text[1]=='X' && (targetClientId==config.getClientId() || config.getClientId()==255)) {
            int clientId = parameter;
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

          // command to set nRF24 send pipe address byte: "A:<value>" where <value> is a single character
          if(text[1]=='A') {
            config.setAddressByte((uint8_t)parameter);
            nRF24Addresses[0][0] = (uint8_t)config.getAddressByte();
          }

          #ifdef LED_TYPE_PWM
          // command to set PWM value: "P:<value>" where <value> is 0-100
          if(text[1]=='P') {
            int pwmValue = parameter;
            if(pwmValue <= 100) {
              config.setPwmValue((uint8_t)pwmValue);
              setPWMDutyCycle((uint8_t)pwmValue);
            }
          }

          // command to set illuminance threshold: "I:<value>" where <value> is in lux
          if(text[1]=='I') {
            int threshold = parameter;
            if(threshold <= 255) {
              config.setIlluminanceThreshold((uint8_t)threshold);
            }
          }
          #endif
        }
      }

      #ifdef LED_TYPE_WS2812
      // command to apply a pattern to the LED strip: "L:<pattern>" where <pattern> is pattern code
      if(text[1]=='L') {
        if(!ledStripPatternEnabled) {
          // initialize LED strip if not already done
          initializeLedStripPattern(true);
          ledStripPatternEnabled = true;
        }
        else {
          stepLedStripPattern();
        }
      }

      // command to set LED count: "N:<value>" where <value> is number of LEDs
      if(text[1]=='N' && strlen(text+1)>2) {
        int ledCount = atoi(text + 3);
        if(ledCount >= 0 && ledCount <= MAX_NUM_LEDS) {
          config.setLedCount((uint8_t)ledCount);
        }
      }
      #endif

      #ifndef MULTI_CLIENT
        // send ready to receive the next command
        radio.stopListening();          // set module as transmitter

        payload[1] = 'L';
        payload[3] = '1';
        payload[4] = 0;
        radio.write( payload,sizeof(payload) );
        radio.txStandBy();              // Wait for the transmission to complete
        delayMicroseconds(10000);

        // Now set the module as receiver and wait for commands
        radio.openReadingPipe(1, nRF24Addresses[1]);
        radio.startListening();   
      #endif
    }

    #if !defined(LED_TYPE_PWM) && !defined(LED_TYPE_WS2812)
    #  // check for pull-up button input as motion sensor 3
      if((PINA & _BV(PA3)) == 0) {
        payload[1] = 'M';
        payload[3] = '3';
        payload[4] = 0;

        radio.stopListening();          // set module as transmitter
        radio.write( payload,sizeof(payload) );
        radio.txStandBy();
        radio.openReadingPipe(1, nRF24Addresses[1]);
        radio.startListening();   

        // wait 500ms for debounce
        for(uint16_t i=0; i<500; i++) {
          delayMicroseconds(1000);
        }
      }

    #endif

    delayMicroseconds(10000);
  }
  radio.stopListening();          // set module as transmitter

  payload[1] = 'L';
  payload[3] = '0';
  payload[4] = 0;
  radio.write( payload,sizeof(payload) );

  radio.txStandBy();              // Wait for the transmission to complete
  delayMicroseconds(10000);
}


/**
 * enter MCU sleep mode. nRF24 module is *not* handled in this function
 */
void enterSleep() {
  GIMSK  |= _BV(PCIE1);                  // Enable Pin Change Interrupts
  GIMSK  |= _BV(PCIE0);                  // Enable Pin Change Interrupts
 
  PCMSK1 |= _BV(PCINT8);                 // Enable pin change interrupt on PIN_MOTION1

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

  PCMSK1 &= ~_BV(PCINT8); 

  sleep_disable();                       // Disable sleep mode
}

// ISR for motion sensor (pin change)
ISR (PCINT0_vect) {  
}
ISR (PCINT1_vect) {  
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
  ADCSRA |= _BV(ADEN);									  // enable ADC
	delayMicroseconds(10000);	             // settle
  ADCSRA |= _BV(ADSC);                    // start a conversion
  while (ADCSRA & (1<<ADSC)){}
  uint16_t reading = (ADCL | (ADCH<<8));	// it's important to read ADCL before ADCH
	ADCSRA &= ~_BV(ADEN);									  // disable ADC again

  double voltage = (double)reading * VREF * (R_VCC + R_GND) / (R_GND * 1024.0); // calculate voltage

  payload[1] = 'V';
  dtostrf(voltage, 4, 2, payload+3);   // convert voltage to string
  radio.write( payload,sizeof(payload) );  // Send data
}




#ifdef LED_TYPE_WS2812

void initializeLedStripPattern(bool enablePattern) {
  uint8_t numLeds = config.getLedCount();

  for(uint8_t i=0; i<numLeds; i++) {
    ledArray[i].r = 0;
    ledArray[i].g = 0;
    ledArray[i].b = 0;

    if (enablePattern) {
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

  #ifdef TRIGGERED
    ws2812_setleds_pin(ledArray, numLeds, _BV(PA7) );           // use only PA7 for data, PA3 is used to power on the DCDC
  #else
    ws2812_setleds_pin(ledArray, numLeds, _BV(PA7) | _BV(PA3)); // use PA7 and PA3 for data
  #endif
}

void stepLedStripPattern() {
  uint8_t numLeds = config.getLedCount();

  // shift array content to the right by one position
  struct cRGB lastLed = ledArray[numLeds - 1];
  for(uint8_t i = numLeds - 1; i > 0; i--) {
    ledArray[i] = ledArray[i - 1];
  }
  ledArray[0] = lastLed;

  #ifdef TRIGGERED
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

#ifdef NEEDS_WAKEUP

/**
 * read illuminance using BH1750 sensor via direct I2C communication
 * @return illuminance in lux
 */
float readAndSendIlluminance() {
  float lux = 0.0;
  
  BH1750 lightMeter(0x23);   // Address 0x23 is the default address of the sensor
  delayMicroseconds(10000);  // ensure enough time for Serial output in case of error

  // increase measurement time for better low-light accuracy
  if (lightMeter.configure(BH1750::ONE_TIME_HIGH_RES_MODE) && lightMeter.setMTreg(100)) {
    delayMicroseconds(50000U); // ensure enough time for measurement
    delayMicroseconds(50000U); // ensure enough time for measurement
    delayMicroseconds(50000U); // ensure enough time for measurement
    delayMicroseconds(50000U); // ensure enough time for measurement
    lux = lightMeter.readLightLevel();
  }

  payload[1] = 'I';
  dtostrf(lux, 1, 0, payload+3);   // convert lux to string
  radio.write( payload,sizeof(payload) );  // Send data

  return lux;
}
#endif