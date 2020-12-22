// Uses RadioHead library: http://www.airspayce.com/mikem/arduino/RadioHead/index.html
// download from: http://www.airspayce.com/mikem/arduino/RadioHead/RadioHead-1.113.zip
// but library needed a patch: comment out line 12 in RHSoftwareSPI.cpp
// default pin assignment in constructor of RHSoftwareSPI


#include <RH_NRF24.h>

#if defined(ESP8266)

#define DEBUG_SERIAL

// (SPI) pins to communicate to nRF204
const int NRF204_PIN_CE   = 4;  // D1 mini D2 = gpio4
const int NRF204_PIN_CSN  = 0;  // D1 mini D3 = gpio0
const int NRF204_PIN_MOSI = 13; // D1 mini D7 = gpio13
const int NRF204_PIN_MISO = 12; // D1 mini D6 = gpio12
const int NRF204_PIN_SCK  = 14; // D1 mini D5 = gpio14

// pushbuttons
const int PIN_BUTTON_ON  = 16; // D1 mini D0 = gpio16
const int PIN_BUTTON_OFF = 5;  // D1 mini D1 = gpio5

#else

#error unsupported platform

#endif

// Singleton instance of the radio driver
RH_NRF24 nrf24(NRF204_PIN_CE, NRF204_PIN_CSN);

// command definitions
const uint8_t CMD_NONE = 0;
const uint8_t CMD_ON   = 1;
const uint8_t CMD_OFF  = 2;

uint8_t command = CMD_NONE;


void setup()
{
  Serial.begin(115200);
  Serial.println("\n\nsender started. Initializing nRF204...");

  // initializting nRF204
  if (!nrf24.init()) {
    Serial.println("init failed");
    return;
  }
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1)) {
    Serial.println("setChannel failed");
    return;
  }
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm)) {
    Serial.println("setRF failed");
    return;
  }
  Serial.println("init done");

  // configure pushbuttons
  pinMode(PIN_BUTTON_ON, INPUT_PULLUP);
  pinMode(PIN_BUTTON_OFF,INPUT_PULLUP);
  delay(200);

  if(digitalRead(PIN_BUTTON_ON) == LOW){ 
    Serial.println("push button power on pressed");
    command = CMD_ON;
  }
  if(digitalRead(PIN_BUTTON_OFF) == LOW){ 
    Serial.println("push button power off pressed");
    command = CMD_OFF;
  }

  if(command == CMD_NONE) {
    Serial.println("no command to send");
  }
  else {
    Serial.println("Sending command 10x");

    // Send a message to nrf24_server
    uint8_t data[1];
    data[0] = command;
  
    // send command for 10 seconds
    for (int i = 0 ; i < 10 ; i++) {
      Serial.print("Sending command: ");
      Serial.println(command);
  
      nrf24.send(data, sizeof(data));
      nrf24.waitPacketSent();
      delay(1000);
    } 
  }

  // sleep again...
  Serial.println("sleeping...");
}


void loop()
{
}
