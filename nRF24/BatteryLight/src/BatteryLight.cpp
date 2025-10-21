/**
 * BatteryLight - Battery-powered light with motion sensor and nRF24L01+ wireless control
 * Uses ATtiny84, PIR motion sensor, and LED driver with PWM brightness control
 * 
 * ATtiny84 pinout:
 * - pin  2: PB0,PCINT8  - Motion sensor 1 input
 * - pin  3: PB1         - CE for nRF24L01
 * - pin  4: PB3,PCINT11 - Motion sensor 2 input - requires fusing PB3 as GPIO
 * - pin  5: PB2         - CS for nRF24L01
 * - pin  6: PA7,OC0B    - PWM output to LED driver
 * - pin  7: PA6,MOSI    - SPI MOSI for nRF24LO1
 * - pin  8: PA5,MISO    - SPI MISO for nRF24LO1
 * - pin  9: PA4,SCK     - SPI SCK for nRF24LO1
 * - pin 10: PA3         - DCDC enable for LED driver
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

#include "Configuration.h"

// resistive divider for battery voltage measurement
const double R_VCC = 470000.0;  // VCC to sense point 470k
const double R_GND = 100000.0;  // sense point to GND 100k
const double VREF  = 1.1;       // internal reference voltage

// nRF24 addresses to use (channel) (5 bytes)
// address 0 is used for writing, address 1 for reading
const byte nRF24Addresses[][6] = {"1clnt","remot"};

const uint8_t nRF24PayloadSize = 16; // max. 32 bytes possible

// global RF24 object and payload buffer
RF24          radio(PB1, PB2);           // create a global RF24 object, CE, CSN
char          payload[nRF24PayloadSize]; // create a payload buffer
char*         payloadText = payload+1;   // pointer to text in payload, skipping first byte (client ID)

// global configuration object
Configuration config;

// Function prototypes
void   initAdc();                          // initialize ADC for battery voltage measurement
double readBatteryVoltage();               // read battery voltage
void   initPWM();                          // initialize PWM on PA7
void   setPWMDutyCycle(uint8_t dutyCycle); // set PWM duty cycle on PA7 (0-100)
void   enterSleep();                       // enter sleep mode


void setup() {

  /**
 * pinout:
 * - pin  2: PB0,PCINT8  - Motion sensor 1 input
 * - pin  3: PB1         - CE for nRF24L01
 * - pin  4: PB3,PCINT11 - Motion sensor 2 input - requires fusing PB3 as GPIO
 * - pin  5: PB2         - CS for nRF24L01
 * - pin  6: PA7,OC0B    - PWM output to LED driver
 * - pin  7: PA6,MOSI    - SPI MOSI for nRF24LO1
 * - pin  8: PA5,MISO    - SPI MISO for nRF24LO1
 * - pin  9: PA4,SCK     - SPI SCK for nRF24LO1
 * - pin 10: PA3         - DCDC enable for LED driver
 * - pin 11: PA2         - IIC SDA for BH1750FVI
 * - pin 12: PA1         - IIC SCL for BH1750FVI
 * - pin 13: PA0,ADC0    - Battery voltage measurement
 */

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

  // send connect message
  strcpy(payloadText,"C:1");
  radio.write( payload,sizeof(payload) );
  radio.txStandBy();                         // Wait for the transmission to complete
  radio.powerDown();                         // Power down the radio immediately after sending

  PORTA &= ~_BV(PA3);                        // Set PA3 low: disables DCDC for LED driver => LED off

  // initialize ADC for battery voltage measurement
  initAdc();
}


void loop() {

  // go to sleep until motion is detected
  enterSleep();

  // enable DCDC and initialize PWM to 100% brightness
  PORTA |= _BV(PA3);            // Set PA3 high: enables DCDC for LED driver
  initPWM();                    // initialize PWM on PA7
  setPWMDutyCycle(config.getPwmValue());         // set PWM to configured brightness

  delay(100);                   // let voltage stabilize
  radio.powerUp();              // Power up the radio
  delay(10);

  // send motion detected message
  radio.stopListening(nRF24Addresses[0]);  // switch to writing on pipe 0
  strcpy(payloadText,"M:1");
  radio.write( payload,sizeof(payload) );

  // measure battery voltage and send
  double voltage = readBatteryVoltage();   // read voltage
  strcpy(payloadText,"V:");
  dtostrf(voltage, 3, 1, payloadText+2);       // convert voltage to string
  radio.write( payload,sizeof(payload) );  // Send data

  // send ready to listen message
  strcpy(payloadText,"L:1");
  radio.write( payload,sizeof(payload) );

  radio.txStandBy();              // Wait for the transmission to complete

  // Now set the module as receiver and wait for commands
  radio.openReadingPipe(1, nRF24Addresses[1]);
  radio.startListening();       

  uint32_t startTime = millis();
  while ( (millis() - startTime) < config.getTimeout() * 1000 ) {
    uint8_t pipe;
    if(radio.available(&pipe)) {
      char text[nRF24PayloadSize] = {0};
      radio.read(&text, sizeof(text));

      // get target client ID from message
      uint8_t targetClientId = text[0];

      if(targetClientId != config.getClientId() && targetClientId != 0) {
        // not for me
        continue;
      }

      // process commands

      // command to set new client ID: "X:<value>" where <value> is 0-255
      if(text[1]=='X' && strlen(text+1)>2) {
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

      // ready to receive the next command
      radio.stopListening();          // set module as transmitter

      strcpy(payloadText,"L:1");
      radio.write( payload,sizeof(payload) );
      radio.txStandBy();              // Wait for the transmission to complete

      // Now set the module as receiver and wait for commands
      radio.openReadingPipe(1, nRF24Addresses[1]);
      radio.startListening();   
    }
    else {
      delay(10); // small delay to avoid busy loop
    }
  }
  radio.stopListening();          // set module as transmitter

  strcpy(payloadText,"L:0");
  radio.write( payload,sizeof(payload) );

  radio.txStandBy();              // Wait for the transmission to complete
  radio.powerDown();              // Power down the radio immediately after sending

  PORTA &= ~_BV(PA3);            // Set PA3 low: disables DCDC for LED driver
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
void initPWM() {
  DDRA |= _BV(PA7);                      // Set PA7 as output

  // configure timer control
  TCCR0A = _BV(WGM00)  | _BV(WGM01) |    // Fast PWM mode with TOP = OxFF
           _BV(COM0B0) |_BV(COM0B1);     // set OC0B on compare match, clear at BOTTOM (inverted PWM)

  TCCR0B = _BV(CS01);                    // Prescaler = 8
           
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

