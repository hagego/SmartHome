/*
  ESP8266 code to control a door of a rabbit hatch based on periodically
  querying an openhab server for commands
*/

// mosquitto 1.3.4 speaks MQTT Version 3.1
// change to MQTT_VERSION MQTT_VERSION_3_1_1 after upgrade to 1.3.5 or higher

#define MQTT_VERSION MQTT_VERSION_3_1

#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Servo.h>



// WiFi Router Login - change these to your router settings
const char* SSID     = "";
const char* password = "";

// MQTT broker IP address
const char* mqtt_server = "192.168.178.27";

// servo positions
const int angleClose = 45;
const int angleOpen  = 135;

// pin definitions (GPIO0-GPIO15 all have internal pull-ups)
int pinServoCtrl   = 4;  // GPIO04 servo control, D2 on D1 mini
int pinServoPower  = 14; // GPIO04 servo power, D5 on D1 mini
int pinManualOpen  = 5;  // GPIO05 connects key to manually open, D1 on D1 mini
int pinManualClose = 12; // GPIO12 connects key to manually close, D6 on D1 mini

// deep sleep period
const long SLEEP_PERIOD = 300000000;



void callback(char* topic, byte* payload, unsigned int length);
void processCmd(String cmd);
void controlServo(int angle);

void setup() {
  // setup serial
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println();
  Serial.println("RabbitHatchDoor started");

  // check local push buttons for manual control
  bool localControl = false;
  char localCmd[10];

  pinMode(pinManualOpen, INPUT_PULLUP);
  pinMode(pinManualClose, INPUT_PULLUP);
  delay(200);
    
  if(digitalRead(pinManualOpen) == LOW){ 
    Serial.println("push button manual open pressed");
    localControl = true;
    strcpy(localCmd,"open");
    processCmd(localCmd);
  }
  if(digitalRead(pinManualClose) == LOW){ 
    Serial.println("push button manual close pressed");
    localControl = true;
    strcpy(localCmd,"close");
    processCmd(localCmd);
  }

  WiFiClient espClient;
  PubSubClient client(espClient);
  
  // Connect to WiFi network
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to SID ");
  Serial.println(SSID);
  WiFi.begin(SSID, password);

  static int COUNTER_MAX = 20;
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED && counter<COUNTER_MAX) {
    delay(500);
    Serial.print(".");
    counter++;
  }

  if(counter>=COUNTER_MAX) {
    Serial.print("Connection failed - sleeping again");
    ESP.deepSleep(SLEEP_PERIOD);
  }
  Serial.println("");
  Serial.print("Connected to WiFi, IP address=");
  Serial.println(WiFi.localIP());


  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
    Serial.print("Attempting MQTT connection to broker at ");
    Serial.println(mqtt_server);
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("rabbithutch/connect", "connected");

      if(localControl) {
        client.publish("rabbithutch/local", localCmd);
      }
      else {
        // subscribe to opic and check for retained publications
        client.subscribe("rabbithutch/door");
      }

      for(int i=0 ; i<10 ; i++) {
        client.loop();
        delay(100);      
      }
      client.disconnect();
      WiFi.disconnect();
    }
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }

    Serial.println("sleeping 60s");
    // wake-up of deep sleep more requires connection between GPIO16 and RST and is actually a reset of the chip
    ESP.deepSleep(SLEEP_PERIOD);
}

void loop() {
}

    
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  char cmd[length+1];
  strncpy(cmd,(char*)payload,length);
  cmd[length] = 0;
  processCmd(cmd);
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

  Serial.print("Unknown servo command: ");
  Serial.println(cmd);
}

void controlServo(int angle) {
  EEPROM.begin(1);
  
  int oldAngle = EEPROM.read(0);

  if(oldAngle>180) {
    oldAngle = 180;
  }

  if(angle==oldAngle) {
    Serial.println("no change in door position");
    return;
  }

  Servo servo;
  servo.attach(pinServoCtrl);
  if(angle>oldAngle) {
    for(int i=oldAngle ; i<angle ; i+=5) {
      servo.write(i);
      delay(100);
    }
  }
  else {
    for(int i=oldAngle ; i>angle ; i-=5) {
      servo.write(i);
      delay(100);
    }
  }
  
  servo.write(angle);
  
  EEPROM.write(0,angle);  
  EEPROM.commit();
}

