#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LiquidCrystal.h>

//instantiating LCD display
const int rs, en, d4, d5, d6, d7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//Defining and instantiating radio and parameters
int CE_PIN = 9;
int CSN_PIN = 10;
RF24 radio(CE_PIN,CSN_PIN);
const byte addresses[][6] = {"00001","00002"};

//Defining joystick pins
#define LR_TIP_PIN A0
#define UD_TIP_PIN A2
#define LR_PROBE_PIN A4
#define UD_PROBE_PIN A5

//joystick limit parameters
int LR_MIN = -100;
int LR_MAX = 100;
int UD_MIN = -100;
int UD_MAX = 100;


void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  //setting up LCD display
  lcd.begin(16,2);
}

void loop() {
  //Sends it to radio for communication
  int joyPos[4];
  int joyPosArray[4];
  joyPos[0]= analogRead(LR_TIP_PIN);
  joyPos[1]= analogRead(UD_TIP_PIN);
  joyPos[2] = analogRead(LR_PROBE_PIN);
  joyPos[3] = analogRead(UD_PROBE_PIN);
  joyPosArray[0]= map(joyPos[0], 0, 1023, -100, 100);
  joyPosArray[1]= map(joyPos[1], 0, 1023, 100, -100);
  joyPosArray[2]= map(joyPos[2], 0, 1023, 100,-100);
  joyPosArray[3]= map(joyPos[3], 0, 1023, -100,100);
  radio.write(&joyPosArray, sizeof(joyPos));
  radio.startListening();
  while (!radio.available()) {
    Serial.println("RADIO NOT AVAILABLE");
  }
  int posMetrics[4];
  radio.read(&posMetrics, sizeof(posMetrics));
  int topServo = posMetrics[0];
  int botServo = posMetrics[1];
  int stepper = posMetrics[2];
  int linActuator = posMetrics[3];
  String message = "Right/Left Flexion: " + String(topServo) + "\n" + "Up/Down Flexion: " + String(botServo) + "\n" + "Rotation: " + String(stepper) + "\n" + "Translation: " + String(linActuator);
  lcd.print(message);
  radio.stopListening();
}