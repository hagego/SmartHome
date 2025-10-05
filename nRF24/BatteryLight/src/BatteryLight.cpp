/**
 * BatteryLight - Battery-powered light with motion sensor and nRF24L01+ wireless control
 * Uses ATtiny84, PIR motion sensor, and LED driver with PWM brightness control
 * 
 * ATtiny84 pinout:
 * - pin  2: PB0,PCINT8  - Motion sensor 1 input
 * - pin  3: PB1         - CE for nRF24L01
 * - pin  4: PB3,PCINT11 - Motion sensor 2 input (with pull-up resistor) - requires fusing PB3 as GPIO
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

const uint32_t LIGHT_ON_MS = 30000;

// resistive divider for battery voltage measurement
const double R_VCC = 470000.0;  // VCC to sense point 470k
const double R_GND = 100000.0;  // sense point to GND 100k
const double VREF  = 1.1;       // internal reference voltage

// nRF24 address to use (channel) (5 bytes)
const byte nRF24Address[6] = "1clnt";

const uint8_t nRF24PayloadSize = 16; // max. 32 bytes possible

// global RF24 object
RF24          radio(PB1, PB2);  // create a global RF24 object, CE, CSN

// Function prototypes
void initAdc();
double readVoltage();
void initPWM();
void setPWMDutyCycle(uint8_t dutyCycle);
void enterSleep();




void setup() {

  /**
 * pinout:
 * - pin  2: PB0,PCINT8  - Motion sensor 1 input
 * - pin  3: PB1         - CE for nRF24L01
 * - pin  4: PB3,PCINT11 - Motion sensor 2 input (with pull-up resistor) - requires fusing PB3 as GPIO
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

  DDRB  &= ~_BV(PB0);            // Set PB0 as input: motion sensor 1
  //PORTB |=  _BV(PB0);            // Enable pull-up resistor on PB0
  //DDRB  |=  _BV(PB1);            // Set PB1 as output: CE for nRF24L01
  DDRB  &= ~_BV(PB3);            // Set PB3 as input: motion sensor 2
  PORTB |=  _BV(PB3);            // Enable pull-up resistor on PB3
  //DDRB  |=  _BV(PB2);            // Set PB2 as output: CS for nRF24L01
  DDRA  |=  _BV(PA7);            // Set PA7 as output: LED driver PWM control

  DDRA  |=  _BV(PA3);            // Set PA3 as output: enables DCDC for LED driver

  radio.begin();

  radio.stopListening();          // set module as transmitter
  radio.setAutoAck(1);            // Ensure autoACK is enabled
  radio.setRetries(5,15);         // Max delay between retries & number of retries
  radio.setPayloadSize(nRF24PayloadSize);       // Set payload size to 16 bytes
  radio.setPALevel(RF24_PA_HIGH); // Set power level to high
  radio.openWritingPipe(nRF24Address); // Write to device address 

  // send connect message
  char buffer[10] = "C:1";
  radio.write( buffer,sizeof(buffer) );
  radio.txStandBy();              // Wait for the transmission to complete

  PORTA &= ~_BV(PA3);            // Set PA3 low: disables DCDC for LED driver

    // initialize ADC
  initAdc();

}

void loop() {

  enterSleep();

  radio.powerUp();                       // Power up the radio


    // enable DCDC and initialize PWM
  PORTA |= _BV(PA3);            // Set PA3 high: enables DCDC for LED driver
  initPWM();
  setPWMDutyCycle(100);

        double voltage = readVoltage(); // read voltage

            // send motion detected message
  char buffer[10] = "M:1";
  radio.write( buffer,strlen(buffer) );


      strcpy(buffer,"V:");
      dtostrf(voltage, 3, 1, buffer+2);     // convert voltage to string
      radio.write( buffer,sizeof(buffer) ); // Send data

      // send ready to listen message
  strcpy(buffer,"L:1");
  radio.write( buffer,strlen(buffer) );

      radio.txStandBy();              // Wait for the transmission to complete


    radio.openReadingPipe(0, nRF24Address);

  radio.setPayloadSize(nRF24PayloadSize);
  radio.startListening();       

  uint32_t startTime = millis();
  while ( (millis() - startTime) < LIGHT_ON_MS ) {
      uint8_t pipe;
      if(radio.available(&pipe)) {

      char text[nRF24PayloadSize] = {0};
      radio.read(&text, sizeof(text));

      int pwmValue = atoi(text);
      if(pwmValue >= 0 && pwmValue <= 100) {
        setPWMDutyCycle((uint8_t)pwmValue); 
      }
    }

    delay(100);
  }
  radio.stopListening();          // set module as transmitter
  radio.setAutoAck(1);            // Ensure autoACK is enabled
  radio.setRetries(5,15);         // Max delay between retries & number of retries
  radio.setPayloadSize(nRF24PayloadSize);       // Set payload size to 16 bytes
  radio.setPALevel(RF24_PA_HIGH); // Set power level to high
  radio.openWritingPipe(nRF24Address); // Write to device address 

  strcpy(buffer,"L:0");
  radio.write( buffer,strlen(buffer) );

  radio.txStandBy();              // Wait for the transmission to complete
  radio.powerDown();     // Power down the radio immediately after sending


  PORTA &= ~_BV(PA3);            // Set PA3 low: disables DCDC for LED driver
}



void enterSleep() {
  GIMSK  |= _BV(PCIE1);                  // Enable Pin Change Interrupts
  GIMSK  |= _BV(PCIE0);                  // Enable Pin Change Interrupts
 
  PCMSK1 |= _BV(PCINT8);            // Enable pin change interrupt on PIN_MOTION1

  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // replaces above statement
  sleep_enable();                        // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
  sei();                    // nRF24 address to use (channel) (5 bytes)
  sleep_cpu();                           // sleep
  // The CPU is now sleeping, and will wake up on interrupt
  // The ISR will be called when the interrupt occurs
  // The CPU will wake up and continue executing from here after ISR
  //cli();                                 // Disable interrupts
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

