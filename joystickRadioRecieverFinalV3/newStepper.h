#ifndef NEWSTEPPER_H
#define NEWSTEPPER_H

#include <Arduino.h>

class Stepper {

  private:
    int enPin; // enable pin
    int stepPin; // step pin
    int dirPin;  // direction pin
    int stepsPerRevolution; // depends on motor and microsteps 
    double currentAngle = 0.00; // Keep track of angle
    int stepperDir = 0; // 0 CW ; 1 CCW
    int microSteps; // 8 by default
    int gearRatio;
    bool hasFinishedRotating = true;
    int steps; // keeps track of steps
    float targetSteps; // converts delta theta to target steps

  public:
    Stepper(int EN_PIN_NUM, int STEP_PIN_NUM, int DIR_PIN_NUM, int microStepNum, double gearRatioNum);
    void singleStep();
    void changeDirection(int chosenDirection);
    void incrementalPos(float deltaTheta, int direction);
    static void incrementalPosMulti(float deltaTheta, int direction, Stepper instances[]);
    bool finishedRotating(float deltaTheta);
    static bool allConditionsMet(bool conditions[]);
    
};

#endif