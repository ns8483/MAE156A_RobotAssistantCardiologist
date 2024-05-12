/*
  LinearActuator.h - Library for controlling Firgelli Micro Pen Actuator with Feedback.

  Created by Noah L. Jones, May 10, 2024.
  Released into the public domain.
*/

#include "LinearActuator.h"
#include "Arduino.h"

LinearActuator* LinearActuator::instancePointer = nullptr;

static void LinearActuator::interruptHandler() {
  if (instancePointer != nullptr) {
        instancePointer->encoderUpdatePos();
    }
}

void LinearActuator::encoderUpdatePos() { // attachInterrupt() function requires a static function as its argument
  static boolean A_prev = false;
  static boolean B_prev = false;
  boolean A_current = digitalRead(encoderAPin) == HIGH;
  boolean B_current = digitalRead(encoderBPin) == HIGH;

  if (A_prev && !A_current && B_prev && B_current) {
    // CW transition: A goes LOW before B
    rotDir = 1; // extending
    position = position + pulseFraction; // add to current position
  } else if (A_prev && A_current && B_prev && !B_current) {
    // CCW transition: B goes LOW before A
    rotDir = 0; // retracting
    position = position - pulseFraction; // subtract from current position
  }

  A_prev = A_current;
  B_prev = B_current;
  //Serial.println(String(A_prev) + String(B_prev));
  //Serial.println(String("Current Position: ") + String(position) + String(" mm\n"));
}

LinearActuator::LinearActuator(int ENA_PIN_NUM, int IN1_PIN_NUM, int IN2_PIN_NUM, int ENCODERA_PIN_NUM, int ENCODERB_PIN_NUM) {
  enAPin = ENA_PIN_NUM;
  in1Pin = IN1_PIN_NUM;
  in2Pin = IN2_PIN_NUM;
  encoderAPin = ENCODERA_PIN_NUM;
  encoderBPin = ENCODERB_PIN_NUM;
  pinMode(enAPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
  pinMode(encoderAPin, INPUT_PULLUP);
  pinMode(encoderBPin, INPUT_PULLUP);
  instancePointer = this; // Set the instance pointer to this object
  attachInterrupt(digitalPinToInterrupt(encoderAPin), interruptHandler, CHANGE); // Check for changes on feedback pins
  attachInterrupt(digitalPinToInterrupt(encoderBPin), interruptHandler, CHANGE);
  while(!isCalibrated()){} // wait until actuator has finished calibrating
}

void LinearActuator::changeDirection(int chosenDirection) {
  if (chosenDirection == 1) { // if chosen to extend
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
        analogWrite(enAPin, power);
        rotDir = 1;
  } else if (chosenDirection == 0) { // if chosen to retarct
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
        analogWrite(enAPin, power);
        rotDir = 0;
  }
}

void LinearActuator::stopActuator() {
  digitalWrite(in1Pin, LOW); // stop
  digitalWrite(in2Pin, LOW);
  analogWrite(enAPin, 0); // disable actuator
}

void LinearActuator::incrementalPos(float deltaX, int direction) {
  if(position-deltaX < 0 && direction == 0){return;} // cancel operation if trying to retract past home
  if(direction == 0){deltaX = -1*deltaX;} // negative deltaX for retraction
  changeDirection(direction); // change direction of actuator to chosen
  while(!finishedTargetSteps(deltaX)) {} // wait for step completion
  stopActuator();
  delay(200); // wait for motor to decelerate 
}

bool LinearActuator::finishedTargetSteps(float deltaX) {
  if(deltaX < 0){decelerationPosError = -1*decelerationPosError;}
  static float oldPosition = position; // set initial position to current position
  static float targetPosition = oldPosition + deltaX - decelerationPosError; // calculate targetPosition 
  // update the statics with new values upon successful prior completion
  if(finishedMoving) {
    oldPosition = position;
    targetPosition = oldPosition + deltaX - decelerationPosError;
    finishedMoving = false;
  }
  //Serial.println("\n3Target: " + String(targetPosition) + "\n Old: " + String(oldPosition) + "\n Current: " + String(position) + "\n Step: " + String(deltaX) + "\n direction: " + String(rotDir) + String("posError: ") + String(decelerationPosError));
  // check to see if target has been met
  if(rotDir == 1) { // if extending
    if(position >= targetPosition){ // if position has reached target position 
      finishedMoving = true; // actuator has reached target and finished
      return 1; // return true
    } else {
      return 0; // actuator still extending return false
    }
  } else if(rotDir == 0) { // if retracting
    if(position <= targetPosition){ // if position has reached target position 
      finishedMoving = true; // actuator has reached target and finished 
      return 1; // return true
    } else {
      return 0; // actuator still retracting return false
    }
  }
}

bool LinearActuator::isCalibrated(){
  changeDirection(0); // retract
  delay(50); // wait for actuator to start moving
  while (!isZerod(position)) { //wait until actuator is zerod
    delay(50);
  }
  changeDirection(1); // add 1 mm offset
  while(!finishedTargetSteps(1.00)) {}; 
  stopActuator(); // stop movement 
  delay(500);
  position = 0; // set home position to zero
  return 1;
}

bool LinearActuator::isZerod(float x) {
  static float old_value = 1; // give x some random placeholder value
  if(x != old_value) {
    // actuator is still detracting
    old_value = x;
    return 0;
  } else {
    // actuator is done detracting
    return 1;
  }
}





