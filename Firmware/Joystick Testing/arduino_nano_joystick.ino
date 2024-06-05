#include <Joystick.h>
#include <SerialTransfer.h>

SerialTransfer joystickDataTransfer;

int tipPin = 13;
int probePin = 12;

Joystick tipJoystick(A0, A1, tipPin);
Joystick probeJoystick(A2, A3, probePin);

void setup() {
  Serial.begin(9600);
}

void loop() {
  tipTransfer();
  probeTransfer();
  delay(100);
}

void tipTransfer() {
  joystickDataTransfer.begin(Serial);
  String tipData = tipJoystick.toString();
  joystickDataTransfer.sendDatum(tipData);
  delay(10);
}

void probeTransfer() {
  joystickDataTransfer.begin(Serial);
  String probeData = probeJoystick.toString();
  joystickDataTransfer.sendDatum(probeData);
  delay(10);
}