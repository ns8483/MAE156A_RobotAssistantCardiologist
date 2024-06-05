/*
  Stepper.cpp - Library for controlling Stepper Motors.
  Provides simultaneously, non-blocking stepper motor control,
  that adjusts for gear ratio discrepancies to maintain synchronized speeds.

  Created by Noah L. Jones, May 10, 2024.
  Released into the public domain.
*/

#include "Stepper.h"
#include "Arduino.h"

// Setup Constructor 
Stepper::Stepper(int EN_PIN_NUM, int STEP_PIN_NUM, int DIR_PIN_NUM, int microStepNum, double gearRatioNum) {
  enPin = EN_PIN_NUM;
  stepPin = STEP_PIN_NUM;
  dirPin = DIR_PIN_NUM;
  microSteps = microStepNum;
  gearRatio = gearRatioNum;
  stepsPerRevolution = 200*microSteps;  // Change this according to your motor's specification
  pinMode(enPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  digitalWrite(enPin,LOW); // Enable TMC2209
}

void Stepper::singleStep(){
  digitalWrite(stepPin, HIGH); // Cycle otor once
  delayMicroseconds(50);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(50);
  if (stepperDir == 0){
    currentAngle = currentAngle - 360.00/(static_cast<double>(stepsPerRevolution)*static_cast<double>(gearRatio)); // update current angle (negative for CW)
  } else{
    currentAngle = currentAngle + 360.00/(static_cast<double>(stepsPerRevolution)*static_cast<double>(gearRatio)); // update current angle (positive for CCW)
  }
  Serial.println("\nCurrent Angle: " + String(currentAngle) + " deg");
}

void Stepper::changeDirection(int chosenDirection) {
  if (chosenDirection == 0) { // CW 
    digitalWrite(dirPin, LOW);
    stepperDir = 0;
  } else if (chosenDirection == 1) { // CCW 
    digitalWrite(dirPin, HIGH);
    stepperDir = 1;
  }
}

void Stepper::incrementalPos(float deltaTheta, int direction){
  changeDirection(direction); // update direction
  while(!finishedTargetSteps(deltaTheta)) {  // do single steps until finished rotating by deltaTheta  
    singleStep();
  }
}

static void Stepper::incrementalPosMulti(float deltaTheta, int direction, Stepper instances[]){
  bool complete[sizeof(instances)]; // create a complete condition for each instance
  float percentComplete[sizeof(instances)]; // create an array of percentages for how close motor is to target  
  float criticalMotorPercentage = 0; // smallest motor percentage
  for (int i = 0; i < sizeof(instances); i++) {
    instances[i].changeDirection(direction); // change direction
    complete[i] = false; // set initial complete conditions to false
    percentComplete[i] = 0.000; // set initial percentComplete values to 0
  }
  while(!allConditionsMet(complete)){ // loop until all rotations are complete
    for (int i = 0; i < sizeof(instances); i++) { // loop through all instances
      if(percentComplete[i] <= criticalMotorPercentage && !complete[i]){ // if motor is furthest from completing and has not yet complete
        if(!instances[i].finishedTargetSteps(deltaTheta)){ // if specific instance has not returned true for finishedRotating()
          instances[i].singleStep(); // make a single step
          percentComplete[i] = instances[i].steps/instances[i].targetSteps;
          criticalMotorPercentage = percentComplete[i]; // update criticalMotorPercentage 
        } else {
          complete[i] = true; // rotation for this instance finished
        }
      } else if(percentComplete[i] >= 1){ // if rotation has gone past completion
          complete[i] = true; // rotation for this instance finished
      }
    }
  }
  //Serial.println("Multi Rotation Finished");
}

bool Stepper::finishedTargetSteps(float deltaTheta) { 
  if(finishedMoving) { 
    steps = 0;
    targetSteps = (deltaTheta/360)*stepsPerRevolution*gearRatio;
    finishedMoving = false;
  }
  Serial.println("\n steps: " + String(steps) + "\ntargetSteps: " + String(targetSteps) + "\ngear ratio" + String(gearRatio));
  if (steps < targetSteps){
    steps ++; //add a step
  }
  if (steps >= targetSteps){
    finishedMoving = true;
    return 1; // finsihed rev
  } else {
    return 0; // not finsihed rev
  }
}

static bool Stepper::allConditionsMet(bool conditions[]) { // method to check if all members in array are true
  for(int i = 0; i < sizeof(conditions); i++) {
    if(!conditions[i]) {
      return false;
    }
  }
  return true;
}
