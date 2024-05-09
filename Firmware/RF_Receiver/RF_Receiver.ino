#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

CE_PIN = 7;
CSN_PIN = 8;

RF24 radio(CE_PIN,CSN_PIN);
const byte addresses[][6] = {"00001","00002"};

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(1, addresses[1]);
  radio.setPALevel(RF24_PA_MIN);
}

void loop() {
  radio.startListening();
  if (radio.available()) {
    while (radio.available){
      int angleV = 0;
      radio.read(&angleV,sizeof(angleV));
    }
    delay(5);
    radio.stopListening();
    char text[32] = "";
    radio.read(&text, sizeof(text));
    Serial.println(text);
  }
}
