/**
 * 
 * @file MotionSensor.cpp
 * @brief Motion Sensor using ATTINY84 and nRF24L01
 */

#define RF24_SPI_SPEED 1000000L // 1MHz SPI speed

#include <avr/sleep.h>
#include <avr/interrupt.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// using locally patched version with SW IIC
// #define SDA_PORT PORTA
// #define SDA_PIN 2
// #define SCL_PORT PORTA
// #define SCL_PIN 1
#include "BH1750.h"



// Pin definitions
const uint8_t PIN_CE  = PIN1;      // PB1, CE  pin for nRF24L01
const uint8_t PIN_CSN = PIN2;      // PB2, CSN pin for nRF24L01

/**
 * PCINT11 can only be used after disabling the external reset pin.
 * This is done by setting the high fuse to 0x5F at last step
 * for this, select "custom_fuses" section in platformio.ini and run Set Fuses command.
 */
const uint8_t PIN_MOTION1 = PCINT8;  // Motion sensor 1 pin AM312
const uint8_t PIN_MOTION2 = PCINT11; // Motion sensor 2 pin AM312
const uint8_t PIN_MOTION3 = PCINT3;  // Motion sensor 3 pin AM312
const uint8_t PIN_VBAT    = PA0;     // Battery voltage pin PA0/ADC0 (pin 13)

/*
 * "global" pin numbers for digitalRead() API mapped to the motion sensors
 */
const uint8_t PIN_MOTION1_READ_NUMBER = PIN0;
const uint8_t PIN_MOTION2_READ_NUMBER = 11;
const uint8_t PIN_MOTION3_READ_NUMBER = PIN7;

// global RF24 object
RF24          radio(PIN_CE, PIN_CSN);  // create a global RF24 object, CE, CSN


// resistive divider for battery voltage measurement
const double R_VCC = 470000.0;  // VCC to sense point 470k
const double R_GND = 100000.0;  // sense point to GND 100k
const double VREF  = 1.1;       // internal reference voltage
 
// nRF24 address to use (channel) (5 bytes)
const byte nRF24Address[6] = "1moti";
//const byte nRF24Address[6] = "hageg";

void enterSleep();


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
 * measure battery voltage
 * @return voltage in V
 */
double readVoltage(){
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
 * read illuminance using BH1750 sensor
 * @return illuminance in lux
 */
double readIlluminance() {
  BH1750 lightMeter(0x23); // Address 0x23 is the default address of the sensor
  delay(10); // ensure enough time for Serial output in case of error

  if (lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE)) {
    delay(200); // ensure enough time for measurement
    return lightMeter.readLightLevel();
  }
  
  return 0.0;
}

/**
 * setup function
 */
void setup() {
  // Set up pins
  DDRB  &= ~_BV(PB0);           // Set PIN_MOTION1 as input w/o pull-up
  PORTB &= ~_BV(PB0);           // Disable pull-up resistor on PIN_MOTION1
  DDRB  &= ~_BV(PB3);           // Set PIN_MOTION2 as input w/o pull-up
  PORTB &= ~_BV(PB3);           // Disable pull-up resistor on PIN_MOTION2

  DDRA  &= ~_BV(PA3);           // Set PIN_MOTION3 as input with pull-up
  PORTA |=  _BV(PA3);           // Enable pull-up resistor on PIN_MOTION3

  // pinMode(PIN_CSN, OUTPUT);    // Set SPI CSN pin as output
  // pinMode(PIN_CE,  OUTPUT);    // Set CE pin as output

  radio.begin();

  radio.stopListening();          // set module as transmitter
  radio.setAutoAck(1);            // Ensure autoACK is enabled
  radio.setRetries(5,15);         // Max delay between retries & number of retries
  radio.setPayloadSize(16);       // Set payload size to 16 bytes
  radio.setPALevel(RF24_PA_HIGH); // Set power level to high
  radio.openWritingPipe(nRF24Address); // Write to device address 

  // send connect message
  char buffer[10] = "C:1";
  radio.write( buffer,sizeof(buffer) );
  radio.txStandBy();              // Wait for the transmission to complete

  // initialize ADC
  initAdc();
}

//
// loop
//
int counter = 0;
void loop() {
  char buffer[10];

  // wait for motion detected
  enterSleep();

  // wake up after motion detected. Send message
  counter++;

  double illuminance = readIlluminance(); // read brightness
  strcpy(buffer,"I:");
  dtostrf(illuminance, 3, 1, buffer+2);     // convert brightness to string
  radio.write( buffer,sizeof(buffer) );     // Send data

  uint8_t pinNumber       = 255;
  uint8_t pinTriggerState = HIGH;
  if(digitalRead(PIN_MOTION1_READ_NUMBER) == HIGH) {
    pinNumber = PIN_MOTION1_READ_NUMBER;
    strcpy(buffer,"M:1");
    radio.write( buffer,sizeof(buffer) );
  }

  if(digitalRead(PIN_MOTION2_READ_NUMBER) == HIGH) {
    pinNumber = PIN_MOTION2_READ_NUMBER;
    strcpy(buffer,"M:2");
    radio.write( buffer,sizeof(buffer) );
  }

  if(digitalRead(PIN_MOTION3_READ_NUMBER) == LOW) {
    pinNumber = PIN_MOTION3_READ_NUMBER;
    pinTriggerState = LOW;
    strcpy(buffer,"M:3");
    radio.write( buffer,sizeof(buffer) );
  }

  // measure battery voltage every 10th time
  if(counter==10) {
    counter = 0;
    double voltage = readVoltage(); // read voltage

    strcpy(buffer,"V:");
    dtostrf(voltage, 3, 1, buffer+2);     // convert voltage to string
    radio.write( buffer,sizeof(buffer) ); // Send data
  }

  radio.txStandBy();     // Wait for the transmission to complete
  radio.powerDown();     // Power down the radio immediately after sending

  if(pinNumber!=255) {
    // wait for the motion sensor to go LOW
    while(digitalRead(pinNumber) == pinTriggerState) {
      delay(100); // wait 100ms
    }
  }
  
  radio.powerUp();       // Power up the radio again
  delay(100);
  strcpy(buffer,"M:0");
  radio.write( buffer,sizeof(buffer) );
  delay(1000);

  radio.txStandBy();     // Wait for the transmission to complete
  radio.powerDown();     // Power down the radio immediately after sending
}

void enterSleep() {
  GIMSK  |= _BV(PCIE1);                  // Enable Pin Change Interrupts
  GIMSK  |= _BV(PCIE0);                  // Enable Pin Change Interrupts

  PCMSK1 |= _BV(PIN_MOTION1);            // Enable pin change interrupt on PIN_MOTION1
  PCMSK1 |= _BV(PIN_MOTION2);            // Enable pin change interrupt on PIN_MOTION2
  PCMSK0 |= _BV(PIN_MOTION3);            // Enable pin change interrupt on PIN_MOTION3

  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // replaces above statement
  sleep_enable();                        // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
  sei();                                 // Enable interrupts
  sleep_cpu();                           // sleep
  // The CPU is now sleeping, and will wake up on interrupt
  // The ISR will be called when the interrupt occurs
  // The CPU will wake up and continue executing from here after ISR
  cli();                                 // Disable interrupts
  GIMSK  &= ~_BV(PCIE1);                 // Disable Pin Change Interrupts
  GIMSK  &= ~_BV(PCIE0);                 // Disable Pin Change Interrupts

  PCMSK1 &= ~_BV(PIN_MOTION1); 
  PCMSK1 &= ~_BV(PIN_MOTION2);           // Disable pin change interrupt on PIN_MOTION1 and PIN_MOTION2
  PCMSK0 &= ~_BV(PIN_MOTION3);           // Disable pin change interrupt on PIN_MOTION3

  sleep_disable();                       // Disable sleep mode
  radio.powerUp();                       // Power up the radio
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

