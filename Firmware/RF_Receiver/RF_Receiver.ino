#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

int CE_PIN = 7;
int CSN_PIN = 8;

RF24 radio(CE_PIN,CSN_PIN);
const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    int joyPosArray[4] = {};
    radio.read(&joyPosArray, sizeof(joyPosArray));
    Serial.println(String("x1: ") + String(joyPosArray[0]) + String("  y1: ") + String(joyPosArray[1]) + String("  x2: ") + String(joyPosArray[2]) + String("  y2: ") + String(joyPosArray[3]));
  }
  delay(200);
}
