/**
 * 
 * @file MotionSensor.cpp
 * @brief Motion Sensor using ATTINY85 and nRF24L01
 * 
 * IMPORTANT: In contrast to the ATtin85 pin out diagram, the pins for SPI are:
 * MISO: Pin 5 (PB0)
 * MOSI: Pin 6 (PB1)
 */

#define RF24_SPI_SPEED 1000000L // 1MHz SPI speed

#include <avr/sleep.h>
#include <avr/interrupt.h>

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

void enterSleep();

 // nRF24 CE/CSN pins
 const uint8_t PIN_CE  = PIN3;
 const uint8_t PIN_CSN = PIN4;

 const uint8_t PIN_MOTION = PIN3; // Motion sensor pin

 RF24         radio(PIN_CE, PIN_CSN);  // create a global RF24 object, CE, CSN
 
 // nRF24 address to use (channel) (5 bytes)
 const byte nRF24Address[6] = "1moti";

int payload = 0;


void setup() {
  // Set up pins
  pinMode(PIN_MOTION, INPUT); // Set motion sensor pin as input
  pinMode(PIN_CSN, OUTPUT);   // Set SPI CSN pin as output
  pinMode(PIN_CE,  OUTPUT);   // Set CE pin as output

  // disable ADC
  

  // setup sleep mode with pin change interrupt
  // enterSleep();

  radio.begin();

  radio.stopListening();          // set module as transmitter
  radio.setAutoAck(1);            // Ensure autoACK is enabled
  radio.setRetries(5,15);         // Max delay between retries & number of retries
  radio.setPayloadSize(16);       // Set payload size to 16 bytes
  radio.setPALevel(RF24_PA_HIGH); // Set power level to high
  radio.openWritingPipe(nRF24Address); // Write to device address 

  delay(100);

}

void loop() {
  
  payload++;
  char buffer[10];
  sprintf(buffer,"%d", payload);
  radio.write( buffer,sizeof(buffer) ); //Send data to 'Receiver' ever second
  radio.txStandBy();                   // Wait for the transmission to complete

  delay(1000); // wait for a second
  
}

void enterSleep() {
  GIMSK  |= _BV(PCIE);                    // Enable Pin Change Interrupts
  PCMSK  |= _BV(PIN_MOTION);              // Use PIN_MOTION as interrupt pin
  ADCSRA &= ~_BV(ADEN);                   // Disable ADC
  radio.powerDown();                     // Power down the radio
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement
  sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
  sei();                                  // Enable interrupts
  sleep_cpu();                            // sleep
  // The CPU is now sleeping, and will wake up on interrupt
  // The ISR will be called when the interrupt occurs
  // The CPU will wake up and continue executing from here after ISR
}

// ISR for motion sensor (pin change)
ISR (PCINT0_vect) {
  cli();                                // Disable interrupts
  GIMSK &= ~(1<<PCIE);                  // Disable Pin Change Interrupts
  PCMSK &= ~_BV(PIN_MOTION);            // Turn off PIN_MOTION as interrupt pin
  sleep_disable();
  radio.powerUp();                      // Power up the radio
}

// ISR for bad interrupt
// This is a catch-all for any interrupts that don't have a specific handler
ISR(BADISR_vect) {
}

