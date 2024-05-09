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

//*************************STEPPER AND LINEAR ACTUATOR CLASSES AND VARIABLES HERE**********************
// Tip Rotation
int EN_TIP_PIN = 10; // Enable
int STEP_TIP_PIN = 9; // Step
int DIR_TIP_PIN = 8; // Direction
int tipGearRatio = 40; //40:1 ratio

// Handle Rotation
int EN_HANDLE_PIN = 7; // Enable
int STEP_HANDLE_PIN = 6; // Step
int DIR_HANDLE_PIN = 5; // Direction
double handleGearRatio = 6.67; //6.67:1 ratio

// Linear Actuator
int ENA_PIN_NUM = 11; // Motor contoller voltage control (PWM)
int IN1_PIN_NUM = 12; // Motor direction
int IN2_PIN_NUM = 13; // Motor direction
int ENCODERA_PIN_NUM = 2; // Hall Sensor digital signal 
int ENCODERB_PIN_NUM = 3; // 90 deg phase difference

// Servos
int Servos1_PIN = 9; // change later

// Global Vars
float linearStepSize = 1.00; // default linear step size [mm]
float rotationStepSize = 1.00; // default rotation step size [deg]
int microSteps = 8; // default number of stepper microsteps 

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
    // Setup Constructor 
    Stepper(int EN_PIN_NUM, int STEP_PIN_NUM, int DIR_PIN_NUM, int microStepNum, double gearRatioNum) {
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

    void singleStep(){
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
  
    void changeDirection(int chosenDirection) {
      if (chosenDirection == 0) { // CW 
        digitalWrite(dirPin, LOW);
        stepperDir = 0;
      } else if (chosenDirection == 1) { // CCW 
        digitalWrite(dirPin, HIGH);
        stepperDir = 1;
      }
    }

    void incrementalPos(float deltaTheta, int direction){
      changeDirection(direction); // update direction
      while(!finishedRotating(deltaTheta)) {  // do single steps until finished rotating by deltaTheta  
        singleStep();
      }
    }

    static void incrementalPosMulti(float deltaTheta, int direction, Stepper instances[]){
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
            if(!instances[i].finishedRotating(deltaTheta)){ // if specific instance has not returned true for finishedRotating()
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

    bool finishedRotating(float deltaTheta) { 
      if(hasFinishedRotating) { 
        steps = 0;
        targetSteps = (deltaTheta/360)*stepsPerRevolution*gearRatio;
        hasFinishedRotating = false;
      }
      //Serial.println("\n steps: " + String(steps) + "\ntargetSteps: " + String(targetSteps) + "\ngear ratio" + String(gearRatio));
      if (steps < targetSteps){
        steps ++; //add a step
      }
      if (steps >= targetSteps){
        hasFinishedRotating = true;
        return 1; // finsihed rev
      } else {
        return 0; // not finsihed rev
      }
    }

    static bool allConditionsMet(bool conditions[]) { // method to check if all members in array are true
      for(int i = 0; i < sizeof(conditions); i++) {
        if(!conditions[i]) {
          return false;
        }
      }
      return true;
    }
};

class LinearActuator;
static LinearActuator *instancePointer; // instance pointer allows indirect access of non-static vars from static methods

class LinearActuator{
  private:
    int enAPin; //enables actuator
    int in1Pin; // direction pin 1
    int in2Pin; // direction pin 2
    int encoderAPin; // one of two sensors on hall sensor
    int encoderBPin; // one of two sensors on hall sensor
    int power = 255; // PWM power sent to motor controller (0-255 min-max)
    float position = 0; // actuator position [mm]
    int rotDir = 0; // 0 = retracting, 1 = extending)
    float pulseFraction = (float) 1/45; // 45 pulses per mm of extension
    volatile boolean A_set = false;
    volatile boolean B_set = false;
    
    static void encoderUpdatePos(int encoder) { // attachInterrupt() function requires a static function as its argument
      static boolean A_prev = false;
      static boolean B_prev = false;

      boolean A_current = digitalRead(instancePointer->encoderAPin) == HIGH;
      boolean B_current = digitalRead(instancePointer->encoderBPin) == HIGH;

      if (A_prev && !A_current && B_prev && B_current) {
        // CW transition: A goes LOW before B
        instancePointer->rotDir = 0; // retracting
        instancePointer->position = instancePointer->position - instancePointer->pulseFraction; // subtract from current position
      } else if (A_prev && A_current && B_prev && !B_current) {
        // CCW transition: B goes LOW before A
        instancePointer->rotDir = 1; // extending
        instancePointer->position = instancePointer->position + instancePointer->pulseFraction; // add to current position
      }

      A_prev = A_current;
      B_prev = B_current;
      //Serial.println(String(A_prev) + String(B_prev));
      Serial.println(String("Current Position: ") + String(instancePointer->position) + String(" mm\n"));
    }

  public:
    // Setup Constructor 
    LinearActuator(int ENA_PIN_NUM, int IN1_PIN_NUM, int IN2_PIN_NUM, int ENCODERA_PIN_NUM, int ENCODERB_PIN_NUM) {
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
      attachInterrupt(digitalPinToInterrupt(encoderAPin), encoderUpdatePos, CHANGE); // Check for changes on feedback pins
      attachInterrupt(digitalPinToInterrupt(encoderBPin), encoderUpdatePos, CHANGE);
      while(!isCalibrated()){} // wait until actuator has finished calibrating
    }

    void changeDirection(int chosenDirection) {
      if (chosenDirection == 1) { // if chosen to extend
            digitalWrite(in1Pin, HIGH);
            digitalWrite(in2Pin, LOW);
            analogWrite(enAPin, 255);
            rotDir = 1;
      } else if (chosenDirection == 0) { // if chosen to retarct
            digitalWrite(in1Pin, LOW);
            digitalWrite(in2Pin, HIGH);
            analogWrite(enAPin, 255);
            rotDir = 0;
      }
    }

    void stopActuator() {
      digitalWrite(in1Pin, LOW); // stop
      digitalWrite(in2Pin, LOW);
      analogWrite(enAPin, 0); // disable actuator
    }

    void incrementalPos(float deltaX, int direction) {
      changeDirection(direction); // change direction of actuator to chosen
      while(!finishedTranslating(deltaX)) {} // wait for step completion
      stopActuator();
    }

    bool finishedTranslating(float deltaX) {
      if(rotDir == 0){deltaX = -1*deltaX;} // make stepSize negative if retarcting
      static bool hasFinished = false; // 
      static float oldPosition = position; // set initial position to current position
      static float targetPosition = oldPosition + deltaX; // calculate targetPosition 
      // update the statics with new values upon successful prior completion
      if(hasFinished) {
        oldPosition = position;
        targetPosition = oldPosition + deltaX;
        hasFinished = false;
      }
      //Serial.println("\n Target: " + String(targetPosition) + "\n Old: " + String(oldPosition) + "\n Current: " + String(position) + "\n Step: " + String(stepSize));
      // check to see if target has been met
      if(rotDir == 1) { // if extending
        if(position >= targetPosition){ // if position has reached target position 
          hasFinished = true; // actuator has reached target and finished
          return 1; // return true
        } else {
          return 0; // actuator still extending return false
      }
      } else if(rotDir == 0) { // if retracting
        if(position <= targetPosition){ // if position has reached target position 
          hasFinished = true; // actuator has reached target and finished 
          return 1; // return true
        } else {
          return 0; // actuator still retracting return false
        }
      }
    }

    bool isCalibrated(){
      digitalWrite(in1Pin, LOW); // retract
      digitalWrite(in2Pin, HIGH);
      analogWrite(enAPin, 255);
      delay(50); // wait for actuator to start moving
      while (!isZerod(position)) { //wait until actuator is zerod
        delay(50);
      }
      position = 0; // set home position to zero
      digitalWrite(in1Pin, LOW); // stop movement
      digitalWrite(in2Pin, LOW);
      delay(100);
      return 1;
    }

    bool isZerod(float x) {
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
};

//setting up stepper and linear actuator by initializing instances
//Stepper stepperTip(EN_TIP_PIN, STEP_TIP_PIN, DIR_TIP_PIN, microSteps, tipGearRatio);
//Stepper stepperHandle (EN_HANDLE_PIN, STEP_HANDLE_PIN, DIR_HANDLE_PIN, microSteps, handleGearRatio);
//Stepper stepperArray[] = {stepperTip, stepperHandle};
//LinearActuator linearActuator(ENA_PIN_NUM, IN1_PIN_NUM, IN2_PIN_NUM, ENCODERA_PIN_NUM, ENCODERB_PIN_NUM);

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
  Serial.begin(9600);
  pos1 = 0; // position will initially be zero
  pos2 = 0;
  calibrate_servo(x, pos1, 9); // calibrate servos and attach each to appropriate pin
  calibrate_servo(y, pos2, 10);
  verify_bt();
}

void loop() {
  Usb.Task();
  //as long as controller is connected so does not waste computing power
  if (Xbox.connected()) {
    //checks if joysticks are in center
    if (Xbox.getAnalogHat(LeftHatX) > 7500 || Xbox.getAnalogHat(LeftHatX) < -7500 || Xbox.getAnalogHat(RightHatX) > 7500 || Xbox.getAnalogHat(RightHatX) < -7500) {
      //need to check if at extremities and fine tune for value at extremity (should not be 7500 will change)
      if (Xbox.getAnalogHat(LeftHatX) > 7500) {
        Serial.println("Right"); //prints left in for debugging joystick purposes
        //Serial.print(Xbox.getAnalogHat(LeftHatX));
        //Serial.print("\t");
        pos1=pos1+1; //moves the position of top servo one a tick to the right to control position of tip
        x.write(pos1);
      }
      if (Xbox.getAnalogHat(LeftHatX) < -7500) {
        Serial.println("Left");
        //Serial.print(Xbox.getAnalogHat(LeftHatY));
        //Serial.print("\t");
        pos1=pos1-1; //moves the position of top servo one a tick to the left to control position of tip
        x.write(pos1);
      }
      if (Xbox.getAnalogHat(RightHatX) > 7500) {
        Serial.println("Right");
        //Serial.print(Xbox.getAnalogHat(RightHatY));
        pos2=pos2+1; //moves the position of bottom servo one a tick to the right to control position of tip
        y.write(pos2);
      }
      if (Xbox.getAnalogHat(RightHatX) < -7500) {
        Serial.print("Left");
        //Serial.print(Xbox.getAnalogHat(RightHatY));
        pos2=pos2-1; //moves the position of bottom servo one a tick to the left to control position of tip
        y.write(pos2);
      }
      Serial.println();
    }
    if (Xbox.getButtonClick(UP))
      //linearActuator.incrementalPos(1, 1); // linear actuator extension
    if (Xbox.getButtonClick(DOWN))
      //linearActuator.incrementalPos(1, 0); // linear actuator retraction
    if (Xbox.getButtonClick(LEFT))
      //Stepper::incrementalPosMulti(1,0,stepperArray); // synchronous stepper movement direction 0 (CCW)
    if (Xbox.getButtonClick(RIGHT))
      //Stepper::incrementalPosMulti(1,1,stepperArray); // synchronous stepper movement direction 1 (CW)
    Serial.println();
  }

}

void verify_bt() {
//  #if !defined(__MIPSEL__)
//  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
//#endif
//  if (Usb.Init() == -1) {
//    Serial.print(F("\r\nOSC did not start"));
//    while (1); //halt
//  }
  while (!Xbox.connected()) {
    Serial.println("Xbox One Controller Not Found!");
    if (Xbox.connected()){break;}
  }
  Serial.println("Xbox One Controller Connected!");
}

void calibrate_servo(Servo s, int pos, int pin) {
  s.attach(pin);
  for (pos = 0; pos <= 180; pos += 1){
    s.write(0);
  }
  int i = 0;
  for (i=0; i<180; i++){
    s.write(i);
    delay(50);
  }
  s.write(90);
  delay(1000);
}