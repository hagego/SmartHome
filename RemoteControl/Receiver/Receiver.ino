// Uses RadioHead library: http://www.airspayce.com/mikem/arduino/RadioHead/index.html
// download from: http://www.airspayce.com/mikem/arduino/RadioHead/RadioHead-1.113.zip
// but library needed a patch: comment out line 12 in RHSoftwareSPI.cpp
// default pin assignment in constructor of RHSoftwareSPI


#include <RHSoftwareSPI.h>
#include <RH_NRF24.h>

//#define DEBUG_LED
 
#if defined (ESP8266)

  #define DEBUG_SERIAL
  const int NRF204_PIN_CE   = 4;  // D1 mini D2 = gpio4
  const int NRF204_PIN_CSN  = 0;  // D1 mini D3 = gpio0
  const int NRF204_PIN_MOSI = 13; // D1 mini D7 = gpio13
  const int NRF204_PIN_MISO = 12; // D1 mini D6 = gpio12
  const int NRF204_PIN_SCK  = 14; // D1 mini D5 = gpio14

  const int PIN_LED   = LED_BUILTIN;
  const int LED_ON    = LOW;
  const int LED_OFF   = HIGH;
  const int PIN_POWER = 5; // D1 mini D1 = gpio5
  
#elif defined (__AVR_ATtiny85__) 

  #include <avr/wdt.h>
  #include <avr/sleep.h>
  #include <avr/interrupt.h>

  const int NRF204_PIN_CE   = 0; // gpio0 = pin 5 on ATtiny85 (maps to D1 mini D2)
  const int NRF204_PIN_CSN  = 2; // gpio2 = pin 7 on ATtiny85 (maps to D1 mini D3)
  const int NRF204_PIN_MOSI = 1; // gpio1 = pin 6 on ATtiny85 (maps to D1 mini D7)
  const int NRF204_PIN_MISO = 4; // gpio4 = pin 3 on ATtiny85 (maps to D1 mini D6)
  const int NRF204_PIN_SCK  = 3; // gpio3 = pin 2 on ATtiny85 (maps to D1 mini D5)

  const int PIN_LED   = 1;       // connected to built-in LED on Digispark boards
  const int LED_ON    = HIGH;
  const int LED_OFF   = LOW;
  const int PIN_POWER = 1;     // option 1: share with MOSI - but will lead to lots of short power on/off
  //const int PIN_POWER = 5;       // option 2: use gpio 5 / pin 1 which is per default used as reset
                                 // need to blow fuses first, see e.g. http://thetoivonen.blogspot.com/2015/12/fixing-pin-p5-or-6-on-digispark-clones.html
                                 // ~/.arduino15/packages/arduino/tools/avrdude/6.3.0-arduino17$ bin/avrdude -C etc/avrdude.conf -P /dev/ttyACM0 -b 19200 -c avrisp -p attiny85 -U hfuse:w:0x5F:m
 
  #define adc_disable() (ADCSRA &= ~(1<<ADEN)) // disable ADC (before power-off)
  
#else

#error unsupported platform

#endif

// use software SPI in order to run on both ESP8266 and ATTiny85
RHSoftwareSPI swSPI;

// instance of the radio driver
RH_NRF24 nrf24(NRF204_PIN_CE,NRF204_PIN_CSN,swSPI);

const uint8_t CMD_NONE = 0;
const uint8_t CMD_ON   = 1;
const uint8_t CMD_OFF  = 2;

const uint8_t HEADER_LENGTH = 3;
const uint8_t HEADER_BYTE   = 0x55;

uint8_t command = CMD_NONE;

 
void setup() 
{
  #ifdef DEBUG_LED
    pinMode(PIN_LED,OUTPUT);
    flashLed(2);
  #endif
  
  #ifdef DEBUG_SERIAL
  Serial.begin(115200);
  Serial.println("\nsetup started");
  #endif

  #if defined (__AVR_ATtiny85__)
    adc_disable();

    wdt_reset();            // Watchdog reset
    wdt_enable(WDTO_8S);    // Watchdog enable Options: 15MS, 30MS, 60MS, 120MS, 250MS, 500MS, 1S, 2S, 4S, 8S
    WDTCR |= _BV(WDIE);     // Interrupts watchdog enable
    sei();                  // enable interrupts
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Sleep Mode: max
  #endif

  pinMode(PIN_POWER,OUTPUT);

  swSPI.setPins(NRF204_PIN_MISO,NRF204_PIN_MOSI,NRF204_PIN_SCK);
  if (nrf24.init()) {
    #ifdef DEBUG_SERIAL
      Serial.println("init success"); 
    #endif

    // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
    nrf24.setChannel(1);
    nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm);

    #ifdef DEBUG_LED
      flashLed(5);
    #endif 
  }
  else {
    #ifdef DEBUG_SERIAL
      Serial.println("init failed"); 
    #endif
  }

  command = CMD_OFF;
}
 
void loop()
{
  #ifdef DEBUG_SERIAL
    Serial.println("starting loop again"); 
  #endif

  // short between NRF204_PIN_MOSI and PIN_POWER on breadboard
  #if defined (ESP8266)
    pinMode(PIN_POWER,INPUT);
    pinMode(NRF204_PIN_MOSI,OUTPUT);
  #endif

  nrf24.setModeRx();
  delay(100);
  
  // available puts chip into RX mode again
  if (nrf24.available())
  {
    #ifdef DEBUG_SERIAL
      Serial.println("data available"); 
    #endif
  
    // Should be a message for us now   
    uint8_t buf[HEADER_LENGTH+1];
    uint8_t len = sizeof(buf);
    
    if (nrf24.recv(buf, &len) && len>0)
    {
      if(len == HEADER_LENGTH+1) {
        boolean match = true;
        for(uint8_t i=0 ; i<HEADER_LENGTH ; i++) {
          if(buf[i]!=HEADER_BYTE) {
            match = false;
          }
        }
        if(match) {
          command = buf[HEADER_LENGTH];
      
          #ifdef DEBUG_SERIAL
            //nrf24.printBuffer("recived data: ", buf, len);
            Serial.print("received command: ");
            Serial.println(command);
          #endif
        }
        else {
          #ifdef DEBUG_SERIAL
            Serial.println("invalid header"); 
          #endif
        }
      }
      else {
        #ifdef DEBUG_SERIAL
          Serial.println("invalid packet size"); 
        #endif
      }
      
      #ifdef DEBUG_LED
        flashLed(3);
      #endif
    }
  }

  // power down the radio again
  nrf24.setModeIdle();

  #if defined (ESP8266)
    // short between NRF204_PIN_MOSI and PIN_POWER on breadboard
    pinMode(PIN_POWER,OUTPUT);
    pinMode(NRF204_PIN_MOSI,INPUT);
  #endif

  if(command==CMD_ON) {
    #ifdef DEBUG_LED
      digitalWrite(PIN_LED, LED_ON);
    #endif
    
    #ifdef DEBUG_SERIAL
      Serial.println("power out enabled"); 
    #endif

    digitalWrite(PIN_POWER,HIGH);
  }
  if(command==CMD_OFF) {
    #ifdef DEBUG_LED
      digitalWrite(PIN_LED, LED_OFF);
    #endif

    #ifdef DEBUG_SERIAL
      Serial.println("power out disabled"); 
    #endif

    digitalWrite(PIN_POWER,LOW);
  }

  #ifdef DEBUG_SERIAL
    Serial.println("Sleeping"); 
  #endif

  #if defined (__AVR_ATtiny85__)
    sleep_enable();
    sleep_cpu();
  #else
    delay(8000);
  #endif
}


void flashLed(int count) {
  #ifdef DEBUG_LED
    for(int loop=0 ; loop<count ; loop++) {
      digitalWrite(PIN_LED, LED_ON);
      delay(100);
      digitalWrite(PIN_LED, LED_OFF);
      delay(100);  
    }
  #endif
}

// Interrupt Service Routine
#if defined (__AVR_ATtiny85__)
  ISR (WDT_vect) {
    WDTCR |= _BV(WDIE);
  }
#endif
