#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

int CE_PIN = 7;
int CSN_PIN = 8;

RF24 radio(CE_PIN,CSN_PIN);
const byte addresses[][6] = {"00001","00002"};

void setup() {  
  radio.begin();
  radio.openWritingPipe(addresses[1]); // 00002
  radio.openReadingPipe(1, addresses[0]); // 00001
  radio.setPALevel(RF24_PA_MIN);
}

void loop() {
  radio.startListening();
  if (radio.available()){
    while (radio.available()){
      String message = " ";
      radio.read(&message, sizeof(message));
      Serial.println(message);
    }
    delay(5);
    radio.stopListening();
    Serial.println("exited");
  }
}
