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
 * - pin 10: PA3         - DCDC enable for LED driver or data pin for WS2812 strip 2
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
#include "light_ws2812.h"

#include "Configuration.h"

// resistive divider for battery voltage measurement
const double R_VCC = 470000.0;  // VCC to sense point 470k
const double R_GND = 100000.0;  // sense point to GND 100k
const double VREF  = 1.1;       // internal reference voltage

// nRF24 addresses to use (channel) (5 bytes)
// address 0 is used for writing, address 1 for reading
const byte nRF24Addresses[][6] = {"1clnt","remot"};

const uint8_t nRF24PayloadSize = 16; // max. 32 bytes possible

// global configuration object
Configuration config;

// global RF24 object and payload buffer
RF24          radio(PB1, PB2);           // create a global RF24 object, CE, CSN
char          payload[nRF24PayloadSize]; // create a payload buffer
char*         payloadText = payload+1;   // pointer to text in payload, skipping first byte (client ID)

// global LED array for WS2812
const uint8_t MAX_NUM_LEDS = 64;
struct cRGB  ledArray[MAX_NUM_LEDS];

// Function prototypes
void   initAdc();                              // initialize ADC for battery voltage measurement
double readBatteryVoltage();                   // read battery voltage
void   initializePWM();                        // initialize PWM on PA7
void   setPWMDutyCycle(uint8_t dutyCycle);     // set PWM duty cycle on PA7 (0-100)
void   initializeLedStrip();                   // initialize WS2812 LED strip
void   applyLedStripPattern(uint8_t pattern) ; // apply LED strip pattern
void   enterSleep();                           // enter sleep mode


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
 * - pin 10: PA3         - DCDC enable for LED driver or data pin for WS2812 strip 2
 * - pin 11: PA2         - IIC SDA for BH1750FVI
 * - pin 12: PA1         - IIC SCL for BH1750FVI
 * - pin 13: PA0,ADC0    - Battery voltage measurement
 */

  // ensure timier0 runs at 1MHz
  #if(F_CPU == 1000000L)
    TCCR0B = _BV(CS01);                  // Prescaler = 8
  #elif(F_CPU == 8000000L)
    TCCR0B = _BV(CS00) | _BV(CS01);      // Prescaler = 64
  #else
    #error "Unsupported F_CPU for PWM initialization"
  #endif

  // set pin modes
  DDRB  &= ~_BV(PB0);            // pin  2: Set PB0 as input: motion sensor 1
  DDRB  |=  _BV(PB1);            // pin  3: Set PB1 as output: CE for nRF24L01
  DDRB  &= ~_BV(PB3);            // pin  4: Set PB3 as input: motion sensor 2
  DDRB  |=  _BV(PB2);            // pin  5: Set PB2 as output: CS for nRF24L01
  DDRA  |=  _BV(PA7);            // pin  6: Set PA7 as output for PWM to LED driver
  DDRA  |=  _BV(PA6);            // pin  7: Set PA6 as output: SPI MOSI for nRF24LO1
  DDRA  &= ~_BV(PA5);            // pin  8: Set PA5 as input: SPI MISO for nRF24LO1
  DDRA  |=  _BV(PA4);            // pin  9: Set PA4 as output: SPI SCK for nRF24LO1
  DDRA  |=  _BV(PA3);            // pin 10: Set PA3 as output: enables DCDC for LED driver
  DDRA  |=  _BV(PA2);            // pin 11: Set PA2 as output: IIC SDA for BH1750FVI
  DDRA  |=  _BV(PA1);            // pin 12: Set PA1 as output: IIC SCL for BH1750FVI

  #ifdef LED_TYPE_PWM
    initializePWM();                     // initialize PWM on PA7
  #endif

  #ifdef LED_TYPE_WS2812
    initializeLedStrip();
  #endif

  // sore client ID in 1st byte of payload
  payload[0] = config.getClientId();

  // initialize nRF24L01
  radio.begin();

  radio.stopListening();                     // set module as transmitter
  radio.setAutoAck(1);                       // Ensure autoACK is enabled
  radio.setRetries(5,15);                    // Max delay between retries & number of retries
  radio.setPayloadSize(nRF24PayloadSize);    // Set payload size to 16 bytes
  radio.setPALevel(RF24_PA_HIGH);            // Set power level to high
  radio.stopListening(nRF24Addresses[0]);    // switch to writing on pipe 0

  // initialize ADC for battery voltage measurement
  initAdc();



  // send connect message
  strcpy(payloadText,"C:1");
  radio.write( payload,sizeof(payload) );

  // measure battery voltage and send
  double voltage = readBatteryVoltage();   // read voltage
  strcpy(payloadText,"V:");
  dtostrf(voltage, 3, 1, payloadText+2);   // convert voltage to string
  radio.write( payload,sizeof(payload) );  // Send data

  radio.txStandBy();                         // Wait for the transmission to complete
}


void loop() {
  #ifdef LED_TYPE_PWM
    radio.powerDown();                         // Power down the radio immediately after sending
    PORTA &= ~_BV(PA3);                        // Set PA3 low: disables DCDC for LED driver => LED off

    // go to sleep until motion is detected
    enterSleep();

    // enable DCDC and initialize PWM to 100% brightness
    PORTA |= _BV(PA3);            // Set PA3 high: enables DCDC for LED driver
    
    setPWMDutyCycle(config.getPwmValue());         // set PWM to configured brightness

    delayMicroseconds(50000U);    // let voltage stabilize
    radio.powerUp();              // Power up the radio
    delayMicroseconds(10000);

    
    // send motion detected message
    radio.stopListening(nRF24Addresses[0]);  // switch to writing on pipe 0
    strcpy(payloadText,"M:1");
    radio.write( payload,sizeof(payload) );

    // measure battery voltage and send
    double voltage = readBatteryVoltage();   // read voltage
    strcpy(payloadText,"V:");
    dtostrf(voltage, 3, 1, payloadText+2);   // convert voltage to string
    radio.write( payload,sizeof(payload) );  // Send data
  #endif

  // send ready to listen message
  strcpy(payloadText,"L:1");
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

      // command to set new client ID: "X:<value>" where <value> is 0-255
      if(text[1]=='X' && strlen(text+1)>2 && (targetClientId==config.getClientId() || config.getClientId()==255)) {
        int clientId = atoi(text + 3);
        if(clientId > 0 && clientId <= 255) {
          // set new client ID and update payload
          config.setClientId((uint8_t)clientId);
          payload[0] = config.getClientId();
        }
      }
      
      // command to set PWM value: "P:<value>" where <value> is 0-100
      if(text[1]=='P' && strlen(text+1)>2) {
        int pwmValue = atoi(text + 3);
        if(pwmValue >= 0 && pwmValue <= 100) {
          config.setPwmValue((uint8_t)pwmValue);
          setPWMDutyCycle((uint8_t)pwmValue);
        }
      }

      // command to set timeout value: "T:<value>" where <value> is in seconds
      if(text[1]=='T' && strlen(text+1)>2) {
        int timeout = atoi(text + 3);
        if(timeout > 0 && timeout <= 3600) { // max 1 hour
          config.setTimeout((uint16_t)timeout);
        }
      }

      // command to set illuminance threshold: "I:<value>" where <value> is in lux
      if(text[1]=='I' && strlen(text+1)>2) {
        int threshold = atoi(text + 3);
        if(threshold >= 0 && threshold <= 255) {
          config.setIlluminanceThreshold((uint8_t)threshold);
        }
      }

      // command to apply a pattern to the LED strip: "L:<pattern>" where <pattern> is pattern code
      if(text[1]=='L' && strlen(text+1)>2) {
        // for PWM, pattern codes are not supported
        #ifdef LED_TYPE_WS2812
          int patternCode = atoi(text + 3);
          if(patternCode >= 0 && patternCode <= 255) {
            // apply pattern
            applyLedStripPattern((uint8_t)patternCode);
          }
        #endif
      }

      // ready to receive the next command
      radio.stopListening();          // set module as transmitter

      strcpy(payloadText,"L:1");
      radio.write( payload,sizeof(payload) );
      radio.txStandBy();              // Wait for the transmission to complete
      delayMicroseconds(10000);

      // Now set the module as receiver and wait for commands
      radio.openReadingPipe(1, nRF24Addresses[1]);
      radio.startListening();   
    }
    else {
      delayMicroseconds(10000); // small delay to avoid busy loop
    }
  }
  radio.stopListening();          // set module as transmitter

  strcpy(payloadText,"L:0");
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
 
  PCMSK1 |= _BV(PCINT8);            // Enable pin change interrupt on PIN_MOTION1

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
 * measure battery voltage on ADC
 * @return voltage in V
 */
double readBatteryVoltage(){
  ADCSRA |= _BV(ADEN);									  // enable ADC
	delay(100);									            // settle
  ADCSRA |= _BV(ADSC);                    // start a conversion
  while (ADCSRA & (1<<ADSC)){}
  uint16_t reading = (ADCL | (ADCH<<8));	// it's important to read ADCL before ADCH
	ADCSRA &= ~_BV(ADEN);									  // disable ADC again

  double voltage = (double)reading * VREF * (R_VCC + R_GND) / (R_GND * 1024.0); // calculate voltage
  return voltage; // return voltage
}


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
  if (dutyCycle > 100) dutyCycle = 100;
  
  // Calculate the compare value based on duty cycle
  uint8_t compareValue = (uint8_t)(((double)dutyCycle/100.0)*255.0);
  OCR0B = compareValue;
}

void   initializeLedStrip() {
    // Initialize WS2812 LED strip
    uint8_t numLeds = 3;

    for(uint8_t i=0; i<numLeds; i++) {
      ledArray[i].r = 0;
      ledArray[i].g = 0;
      ledArray[i].b = 0;
    }
    ws2812_setleds(ledArray, numLeds);
}

void applyLedStripPattern(uint8_t pattern) {
    uint8_t numLeds = 3;
    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

    switch(pattern) {
      case 1: // red
        r = 255; g = 0; b = 0;
        break;
      case 2: // green
        r = 0; g = 255; b = 0;
        break;
      case 3: // blue
        r = 0; g = 0; b = 255;
        break;
      case 4: // white
        r = 255; g = 255; b = 255;
        break;
      default: // off
        r = 0; g = 0; b = 0;
        break;
    }

    for(uint8_t i=0; i<numLeds; i++) {
      ledArray[i].r = r;
      ledArray[i].g = g;
      ledArray[i].b = b;
    }
    //ws2812_setleds(ledArray, numLeds);
    ws2812_setleds_pin(ledArray, numLeds, _BV(PA7) | _BV(PA3)); // use PA7 and PA3 for data
}