#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//Defining and instantiating radio and parameters
int CE_PIN = 9;
int CSN_PIN = 10;
RF24 radio(CE_PIN,CSN_PIN);
const byte address[6] = "00001";

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
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
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
  delay(200);
}