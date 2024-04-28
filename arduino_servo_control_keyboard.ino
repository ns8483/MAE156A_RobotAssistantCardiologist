#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
int val; //initial value of input

void setup() {
  Serial.begin(9600);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
  if(Serial.available()){
    val = Serial.read(); // read serial value
    if (val == 'd'){ //using wasd to control the motor for now until new arduino comes
      pos += 10;
      myservo.write(pos);
      delay(15);
    }
    if (val == 'a'){
      pos -= 10;
      myservo.write(pos);
      delay(15);
    }
  }
}