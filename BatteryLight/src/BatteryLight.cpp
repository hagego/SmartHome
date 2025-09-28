#include <Arduino.h>

/**
 * Initialize PWM for PA7 (ATtiny84 pin 6), connected to OC0B (Output Compare B for 8-bit Timer0)
 */
void initPWM() {
  DDRA |= _BV(PA7);                      // Set PA7 as output

  // configure timer control registers
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

void setup() {

  DDRB  &= ~_BV(PB0);            // Set PB0 as input: motion sensor
  PORTB &= ~_BV(PB0);            // Disable pull-up resistor on PB0
  DDRA  |=  _BV(PA3);            // Set PA3 as output: enables DCDC for LED driver
  DDRA  |=  _BV(PA7);            // Set PA7 as output: LED driver PWM control

  // disable DCDC for LED driver and wait 5 seconds
  PORTA &= ~_BV(PA3);            // Set PA3 low: disables DCDC for LED driver
  delay(5000);

  // enable DCDC and initialize PWM
  PORTA |= _BV(PA3);            // Set PA3 high: enables DCDC for LED driver
  initPWM();
}

void loop() {
  // Fade in: Increase duty cycle from 0% to 100%
  for (uint8_t dutyCycle = 0; dutyCycle <= 50; dutyCycle += 5) {
    setPWMDutyCycle(dutyCycle);
    delay(100);
  }
  
  delay(1000);
  
  // // Fade out: Decrease duty cycle from 100% to 0%
  for (uint8_t dutyCycle = 50; dutyCycle > 0; dutyCycle -= 5) {
    setPWMDutyCycle(dutyCycle);
    delay(100);
  }

  delay(1000);
}
