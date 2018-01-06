#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <Servo.h>

/*
  ESP8266 code to control a door of a rabbit hatch based on periodically
  querying an openhab server for commands
*/


// WiFi Router Login - change these to your router settings
const char* SSID     = "SID";
const char* password = "PASSWORD";

const int angleClose = 45;
const int angleOpen  = 135;

// pin definitions
int pinServo = 2; // GPIO02 controls servo
Servo servo;


void processCmd(String cmd);
void controlServo(int angle);

void setup() {
  // setup serial
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println("RabbitHatchDoor started");


  // Connect to WiFi network
  Serial.println();
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to SID ");
  Serial.println(SSID);
  WiFi.begin(SSID, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi, IP address=");
  Serial.println(WiFi.localIP());

  EEPROM.begin(1);
  servo.attach(pinServo);
  int oldAngle = EEPROM.read(0);
  if(oldAngle>180) {
    oldAngle = 180;
  }
  servo.write(oldAngle);
}

void loop() {
  // connect to server
  Serial.println("Connecting...");

  WiFiClient client;
  const int httpPort = 3457;
  const char* host="192.168.178.27";
  
  if (!client.connect(host, httpPort)) {
    Serial.println("connection failed");
  }
  else {
    // This will send the request to the server
    Serial.println("sending ping");
    client.println("ping");

    while (client.connected())
    {
      if (client.available())
      {
        String line = client.readStringUntil('\n');
        Serial.println(line);

        if(line.length()>0) {
          processCmd(line);
          Serial.println("closing connection");
          client.stop();
        }
      }
    }
  }
    
  Serial.println("sleeping 10s");
  // wake-up of deep sleep more requires connection between GPIO16 and RST and is actually a reset of the chip
  // ESP.deepSleep(10000000);
  delay(10000);
}

void processCmd(String cmd) {
  if(cmd == "nothing") {
    Serial.println("no action needed");
    return;
  }

  if(cmd == "open") {
    Serial.println("received door open");
    controlServo(angleOpen);

    return;
  }

  if(cmd == "close") {
    Serial.println("received door close");
    controlServo(angleClose);
    
    return;
  }

  Serial.print("Unknown command: ");
  Serial.println(cmd);
}

void controlServo(int angle) {
  int oldAngle = EEPROM.read(0);

  if(oldAngle>180) {
    oldAngle = 180;
  }

  if(angle>oldAngle) {
    for(int i=oldAngle ; i<angle ; i+=10) {
      servo.write(i);
      delay(100);
    }
  }
  else {
    for(int i=oldAngle ; i>angle ; i-=10) {
      servo.write(i);
      delay(100);
    }
  }
  servo.write(angle);
  
  EEPROM.write(0,angle);  
  EEPROM.commit();
}

