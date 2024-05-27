#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

int CE_PIN = 7;
int CSN_PIN = 8;

RF24 radio(CE_PIN,CSN_PIN);
const byte address[6] = "00001";

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(address); // 00001
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
}

void loop() {
  char text[32] = "Hello Worl";
  radio.write(&text, sizeof(text));
  delay(5);
  Serial.println(text);
}