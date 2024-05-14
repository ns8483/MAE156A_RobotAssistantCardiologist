#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

int CE_PIN = 9;
int CSN_PIN = 10;

RF24 radio(7,8);
const byte address[6] = "00001";

void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  Serial.println("hi");
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    Serial.println(text);
  }
  delay(200);
}
