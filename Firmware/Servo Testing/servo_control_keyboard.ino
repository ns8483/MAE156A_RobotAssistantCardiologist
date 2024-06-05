#include <Servo.h>
Servo myservo;
int pos = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("-------------------------");
  Serial.println("ARos   is loading....");
  delay(1000);
  Serial.println("ARos loaded succesfully");
  Serial.println("-------------------------");
  myservo.attach(9);
  Serial.println("calibrating   servo...");
  for (pos = 0; pos <= 180; pos += 1){
    myservo.write(0);
  }
  delay(1000);
  myservo.write(180);
  delay(1000);
  myservo.write(90);
  delay(1000);
  Serial.println("servo   calibrated");
  Serial.println("-------------------------");
  Serial.println("Comand   input online, write command to perform action");
  Serial.println("-------------------------");
}

void loop() {

  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'd'){
      if (pos != 180){
        pos = pos + 1;
        myservo.write(pos);
        Serial.println(pos);
      }
      else {
        Serial.println("cannot execute command");
      }
    }
    if (command == 'a'){
      if (pos != 0){
        pos = pos - 1;
        myservo.write(pos);
        Serial.println(pos);
      }
      else {
        Serial.println("cannot execute command");
      }
    }
  }
}
