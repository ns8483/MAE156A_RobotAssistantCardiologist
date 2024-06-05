#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//Defining and instantiating radio and parameters
int CE_PIN = 7;
int CSN_PIN = 8;
RF24 radio(CE_PIN,CSN_PIN);
const byte address[6] = "00001";

//Defining joystick pins
#define LR_TIP_PIN A0
#define UD_TIP_PIN A1
#define LR_PROBE_PIN A2
#define UD_PROBE_PIN A3

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
  joyPos[0]= analogRead(LR_TIP_PIN);
  joyPos[1] = analogRead(UD_TIP_PIN);
  joyPos[2] = analogRead(LR_PROBE_PIN);
  joyPos[3] = analogRead(UD_PROBE_PIN);
  radio.write(&joyPos, sizeof(joyPos));
  delay(200);
}