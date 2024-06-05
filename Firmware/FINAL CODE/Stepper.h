/*
  Stepper.h - Library for controlling Stepper Motors.
  Provides simultaneously, non-blocking stepper motor control,
  that adjusts for gear ratio discrepancies to maintain synchronized speeds.

  Created by Noah L. Jones, May 10, 2024.
  Released into the public domain.
*/
#ifndef Stepper_h
#define Stepper_h

#include "Arduino.h"

class Stepper {
  private:
    int enPin; // enable pin
    int stepPin; // step pin
    int dirPin;  // direction pin
    int stepsPerRevolution; // depends on motor and microsteps 
    int microSteps; // 8 by default
    int gearRatio;
    int steps; // keeps track of steps
    float targetSteps; // converts delta theta to target steps

  public:
    double currentAngle = 0.00; // Keep track of angle
    int stepperDir = 0; // 0 CW ; 1 CCW
    bool finishedMoving = true;
    // Setup Constructor 
    Stepper(int EN_PIN_NUM, int STEP_PIN_NUM, int DIR_PIN_NUM, int microStepNum, double gearRatioNum);
    void singleStep();
    void changeDirection(int chosenDirection);
    void incrementalPos(float deltaTheta, int direction);
    static void incrementalPosMulti(float deltaTheta, int direction, Stepper instances[]);
    bool finishedTargetSteps(float deltaTheta);
    static bool allConditionsMet(bool conditions[]);
    
};

#endif