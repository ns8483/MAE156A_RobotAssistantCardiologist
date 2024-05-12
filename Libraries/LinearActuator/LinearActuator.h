/*
  LinearActuator.h - Library for controlling Firgelli Micro Pen Actuator with Feedback.

  Created by Noah L. Jones, May 10, 2024.
  Released into the public domain.
*/
#ifndef LinearActuator_h
#define LinearActuator_h

#include "Arduino.h"

class LinearActuator{
  private:
    int enAPin; //enables actuator
    int in1Pin; // direction pin 1
    int in2Pin; // direction pin 2
    int encoderAPin; // one of two sensors on hall sensor
    int encoderBPin; // one of two sensors on hall sensor
    int rotDir = 0; // 0 = retracting, 1 = extending)
    float decelerationPosError = 0.67; // overshoot due to motor decelerating at 255 PWM (assumes motor has reached full speed)
    float pulseFraction = (float) 1/45; // 45 pulses per mm of extension
    volatile boolean A_set = false;
    volatile boolean B_set = false;
    static LinearActuator* instancePointer; // instance pointer allows indirect access of non-static vars from static methods
    static void interruptHandler();
    void encoderUpdatePos();

  public:
    float position = 0; // actuator position [mm]
    int power = 255; // PWM power sent to motor controller (0-255 min-max)
    static bool finishedMoving = false;  
    // Setup Constructor 
    LinearActuator(int ENA_PIN_NUM, int IN1_PIN_NUM, int IN2_PIN_NUM, int ENCODERA_PIN_NUM, int ENCODERB_PIN_NUM);
    void changeDirection(int chosenDirection);
    void stopActuator();
    void incrementalPos(float deltaX, int direction);
    bool finishedTranslating(float deltaX);
    bool isCalibrated();
    bool isZerod(float x);
};

#endif