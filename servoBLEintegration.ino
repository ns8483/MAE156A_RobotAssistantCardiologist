/*
 very rough version of bluetooth double servo control with arduino uno
 can recognize when buttons are pressed
 need to test with hardware
 */

#include <XBOXONESBT.h>
#include <usbhub.h>
#include <Servo.h>
Servo x;
Servo y;

//position of servo
int pos1;
int pos2;

// Satisfy the IDE
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so

/* You can create the instance of the XBOXONESBT class in two ways */
// This will start an inquiry and then pair with the Xbox One S controller - you only have to do this once
// You will need to hold down the Sync and Xbox button at the same time, the Xbox One S controller will then start to blink rapidly indicating that it is in pairing mode
XBOXONESBT Xbox(&Btd, PAIR);

// After that you can simply create the instance like so and then press the Xbox button on the device
//XBOXONESBT Xbox(&Btd);

void setup() {
  pos1 = 0;
  pos2 = 0;
  x.write(pos1); //sets servo to neutral position right away if before attached to pin
  y.write(pos2);
  x.attach(9); // servo 1 is attached to pin 9 of the arduino
  y.attach(10); //servo 2 is atttached to pin 10 of the arduino
  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXbox One S Bluetooth Library Started"));
}
void loop() {
  Usb.Task();

  if (Xbox.connected()) {
    //source code used 7500 and -7500 for xbox joystick values, need to test and fine tune
    if (Xbox.getAnalogHat(LeftHatX) > 7500 || Xbox.getAnalogHat(LeftHatX) < -7500 || Xbox.getAnalogHat(RightHatX) > 7500 || Xbox.getAnalogHat(RightHatX) < -7500) {
      if (Xbox.getAnalogHat(LeftHatX) > 7500) {
        Serial.print(F("LeftHatX: "));
        Serial.print(Xbox.getAnalogHat(LeftHatX));
        Serial.print("\t");
        pos1=pos1+10;
        x.write(pos1);
      }
      if (Xbox.getAnalogHat(LeftHatX) < -7500) {
        Serial.print(F("LeftHatY: "));
        Serial.print(Xbox.getAnalogHat(LeftHatY));
        Serial.print("\t");
        pos1=pos1-10;
        x.write(pos1);
      }
      if (Xbox.getAnalogHat(RightHatX) > 7500) {
        Serial.print(F("RightHatX: "));
        Serial.print(Xbox.getAnalogHat(RightHatY));
        pos2=pos2+10;
        y.write(pos2);
      }
      if (Xbox.getAnalogHat(RightHatX) < -7500) {
        Serial.print(F("RightHatY: "));
        Serial.print(Xbox.getAnalogHat(RightHatY));
        pos2=pos2-10;
        y.write(pos2);
      }
      Serial.println();
    }
    Serial.println();
  }
}