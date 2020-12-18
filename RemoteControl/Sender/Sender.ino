// Uses RadioHead library: http://www.airspayce.com/mikem/arduino/RadioHead/index.html
// download from: http://www.airspayce.com/mikem/arduino/RadioHead/RadioHead-1.113.zip
// but library needed some patches, use version stored together with this sketch

#include <SPI.h>
#include <RH_NRF24.h>

#if defined(ESP8266)

#define DEBUG_SERIAL
const int NRF204_PIN_CE   = 4;  // D1 mini D2 = gpio4
const int NRF204_PIN_CSN  = 0;  // D1 mini D3 = gpio0
const int NRF204_PIN_MOSI = 13; // D1 mini D7 = gpio13
const int NRF204_PIN_MISO = 12; // D1 mini D6 = gpio12
const int NRF204_PIN_SCK  = 14; // D1 mini D5 = gpio14
// MOSI = D7 MISO = D6 SCK  = D5
#else

#error unsupported platform

#endif

// Singleton instance of the radio driver
RH_NRF24 nrf24(NRF204_PIN_CE, NRF204_PIN_CSN);

uint8_t command = 1;


void setup()
{
  Serial.begin(115200);
  Serial.println("started");

  Serial.println("init");
  if (!nrf24.init())
    Serial.println("init failed");

  Serial.println("init done");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");

  Serial.println("setup done");
}


void loop()
{
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

  if (command == 0) {
    command = 1;
  }
  else {
    command = 0;
  }

  // sleep for 30s
  Serial.println("sleeping...");
  delay(30000);
}
