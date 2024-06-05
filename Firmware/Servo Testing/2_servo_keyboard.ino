/*
 very rough version of bluetooth double servo control with arduino uno
 can recognize when buttons are pressed
 need to test with hardware
 */

#include <XBOXONESBT.h> // importing xbox and bluetooth libraries
#include <usbhub.h>
#include <Servo.h>
Servo x; //initializing two instances of servo here
Servo y;

//position of servo defined here
int pos1;
int pos2;


void setup() {
  pos1 = 0; // position will initially be zero
  pos2 = 0;
  Serial.begin(9600);
  while (!Serial);
  Serial.println("-------------------------");
  Serial.println("ARos   is loading....");
  delay(1000);
  Serial.println("ARos loaded succesfully");
  Serial.println("-------------------------");
  calibrate_servo(x, pos1, 9); // calibrate servos and attach each to appropriate pin
  calibrate_servo(y, pos2, 10);
  Serial.println("-------------------------");
  Serial.println("Comand   input online, write command to perform action");
  Serial.println("-------------------------");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'd'){
      if (pos1 != 180){
        pos1 = pos1 + 1;
        x.write(pos1);
        Serial.println(pos1);
      }
      else {
        Serial.println("cannot execute command");
      }
    }
    if (command == 'a'){
      if (pos1 != 0){
        pos1 = pos1 - 1;
        x.write(pos1);
        Serial.println(pos1);
      }
      else {
        Serial.println("cannot execute command");
      }
    }
    if (command == 'w'){
      if (pos2 != 180){
        pos2 = pos2 + 1;
        y.write(pos2);
        Serial.println(pos2);
      }
      else {
        Serial.println("cannot execute command");
      }
    }
    if (command == 's'){
      if (pos2 != 0){
        pos2 = pos2 - 1;
        y.write(pos2);
        Serial.println(pos2);
      }
      else {
        Serial.println("cannot execute command");
      }
    }
    
  }
}

void calibrate_servo(Servo s, int pos, int pin) {
  s.attach(pin);
  Serial.println("calibrating   servo...");
  for (pos = 0; pos <= 180; pos += 1){
    s.write(0);
  }
  delay(1000);
  s.write(180);
  delay(1000);
  s.write(90);
  delay(1000);
  Serial.println("servo   calibrated");
}