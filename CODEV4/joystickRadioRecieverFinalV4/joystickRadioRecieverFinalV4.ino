// Libraries
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "LinearActuator.h"
#include "Stepper.h"
// Tip Rotation
int EN_TIP_PIN = 10; // Enable
int STEP_TIP_PIN = 9; // Step
int DIR_TIP_PIN = 13; // Direction
int tipGearRatio = 40; //40:1 ratio
// Handle Rotation
int EN_HANDLE_PIN = 4; // Enable
int STEP_HANDLE_PIN = 6; // Step
int DIR_HANDLE_PIN = 5; // Direction
double handleGearRatio = 6.67; //6.67:1 ratio
// Linear Actuator
int ENA_PIN_NUM = 24; // Motor contoller voltage control (PWM) (change)
int IN1_PIN_NUM = 26; // Motor direction (change)
int IN2_PIN_NUM = 28; // Motor direction (change)
int ENCODERA_PIN_NUM = 3; // Hall Sensor digital signal 
int ENCODERB_PIN_NUM = 2; // 90 deg phase difference
// Knob Rotation
int SERVO_TOP_PIN = 12; // servo for top knob (change)
int SERVO_BOT_PIN = 11; // servo for bottom knob (change)
int servoTopPos = 90; // default position
int prevServoTopPos = servoTopPos; // keeps track of previous servo position
int servoBotPos = 90; // default position
int prevServoBotPos = servoBotPos; // keeps track of previous servo position
// Multiplane Angle Control
int MULTIPLANE_PIN = 44;
int critAnglePos = 0; //CHANGE THESE VALUES OR IT WON'T WORK MIGHT BREAK**************************************
int critAngleNeg = 180;
int neutralAngle = 90;
int multiPlaneMark = 0; //0 means that currently is at neutral, 1 means that currently at CCW mark, -1 means that at CW mark
// NRF Transceiver
int NRF_CSN_PIN = 8;
int NRF_CE_PIN = 7;
int NRF_SCK_PIN = 52;
int NRF_MOSI_PIN = 51;
int NRF_MISO_PIN = 50;
const byte addresses[][6] = {"00001", "00002"};
// Global Vars
float linearStepSize = 1.00; // default linear step size [mm]
float rotationStepSize = 1.00; // default rotation step size [deg]
float servoStepSize = 1.00; // default knob rotation step size [deg]
int microSteps = 8; // default number of stepper microsteps
float joyCritical = 80.0; // Joystick critical actuation value after mapping
float joyPosArray[6] = {}; // [0]: x-right joystick [1]: y-right joystick [2]: x-left joystick [3]: y-left joystick
float posMetrics[4] = {}; // [0]: top servo, [1]: bottom servo, [2]: stepper motor, [3]: linear actuator
//Global objects
Servo servoTop; // top servo object creation
Servo servoBot; // bottom servo object creation
Servo multiplaneServo; //multiplane angle control servo
RF24 radio(NRF_CE_PIN,NRF_CSN_PIN); // radio object creation

//keeps track of what was changed so that can keep track for backtracking.
float prevPos[4] = {90,90,0,0};
float savedPos[4] = {90,90,0,0};

void setup() {
  Serial.begin(115200);
  Serial.println("Calibrating...\n");
  // servo setup
  for(int i=0;i<=servoTopPos;i++){
    servoTop.write(servoTopPos); // send servos to home right away so that will do when attached faster
    servoBot.write(servoBotPos);
    delay(20);
  }
  servoTop.attach(SERVO_TOP_PIN); // attach feedback pins to servo objects
  servoBot.attach(SERVO_BOT_PIN);
  multiplaneServo.attach(MULTIPLANE_PIN);
  // radio setup
  radio.begin();
  radio.openReadingPipe(1, addresses[1]); // 00002
  radio.openWritingPipe(addresses[0]);
  radio.setPALevel(RF24_PA_MAX);
}

void loop(){
  // initialize class objects
  Stepper stepperTip(EN_TIP_PIN, STEP_TIP_PIN, DIR_TIP_PIN, microSteps, tipGearRatio);
  Stepper stepperHandle (EN_HANDLE_PIN, STEP_HANDLE_PIN, DIR_HANDLE_PIN, microSteps, handleGearRatio);
  Stepper stepperArray[] = {stepperTip, stepperHandle};
  LinearActuator linearActuator(ENA_PIN_NUM, IN1_PIN_NUM, IN2_PIN_NUM, ENCODERA_PIN_NUM, ENCODERB_PIN_NUM);
  Serial.println("Calibrating Complete!");
  while(true) {
    radio.startListening();
    delay(5);
    if (radio.available()) {
      //Serial.println("connected");
      radio.read(&joyPosArray, sizeof(joyPosArray));
      delay(5);
      //Serial.println(String("x1: ") + String(joyPosArray[0]) + String("  y1: ") + String(joyPosArray[1]) + String("  x2: ") + String(joyPosArray[2]) + String("  y2: ") + String(joyPosArray[3]));
      if (joyPosArray[0] <= -1*joyCritical){
        if (servoTopPos != 0){
          servoTopPos = servoTopPos - servoStepSize;
          servoTop.write(servoTopPos);
          delay(200); //wait for servo to reach this position
          radio.flush_rx();
          prevPos[0] = servoTopPos;
        }
      }else if (joyPosArray[0] >= joyCritical){
        if (servoTopPos != 180){
          servoTopPos = servoTopPos + servoStepSize;
          servoTop.write(servoTopPos);
          delay(200); //wait for servo to reach this position
          radio.flush_rx();
          prevPos[0] = servoTopPos;
        }
      }else if (joyPosArray[1] <= -1*joyCritical){
        if (servoBotPos != 0){
          servoBotPos = servoBotPos - servoStepSize;
          servoBot.write(servoBotPos);
          delay(200); //wait for servo to reach this position
          radio.flush_rx();
          prevPos[1] = servoBotPos;
        }
      }else if (joyPosArray[1] >= joyCritical){
        if (servoBotPos != 180){
          servoBotPos = servoBotPos + servoStepSize;
          servoBot.write(servoBotPos);
          delay(200); //wait for servo to reach this position
          radio.flush_rx();
          prevPos[1] = servoBotPos;
        }
      }else if (joyPosArray[2] >= joyCritical){ // if left joystick points right and stepper is not already moving
        Stepper::incrementalPosMulti(rotationStepSize,0,stepperArray); // synchronous stepper movement direction 1 (CW)
        radio.flush_rx();
        prevPos[2] = stepperArray[0].currentAngle;
      }else if (joyPosArray[2] <= -1*joyCritical){ // if left joystick points left and stepper is not already moving
        Stepper::incrementalPosMulti(rotationStepSize,1,stepperArray); // synchronous stepper movement direction 0 (CCW)
        radio.flush_rx();
        prevPos[2] = stepperArray[0].currentAngle;
      }else if (joyPosArray[3] >= joyCritical){ // if left joystick points up
        linearActuator.incrementalPos(linearStepSize, 1); // linear actuator extension
        radio.flush_rx();
        prevPos[3] = linearActuator.position;
      }else if (joyPosArray[3] <= -1*joyCritical){ // if left joystick points down
        linearActuator.incrementalPos(linearStepSize, 0); // linear actuator retraction
        radio.flush_rx();
        prevPos[3] = linearActuator.position;
      }
      //look in transmitter code for explanation of this element
      else if (joyPosArray[4] == 1){
        servoStepSize = servoStepSize + 1;
      } else if (joyPosArray[4] == -1) {
        servoStepSize = servoStepSize - 1;
      } else if (joyPosArray[4] == 2) {
        rotationStepSize = rotationStepSize + 1;
      } else if (joyPosArray[4] == -2) {
        rotationStepSize = rotationStepSize - 1;
      } else if (joyPosArray[4] == 3) {
        linearStepSize = 5*linearStepSize;
      } else if (joyPosArray[4] == -3) {
        linearStepSize = 0.5*linearStepSize;
      } //need to calculate critical angle that button is pressed at and update code here **************************************************************************
      else if (joyPosArray[5] == 1) {
        if (multiPlaneMark != 1) {
          if (multiPlaneMark ==0) {
            multiplaneServo.write(critAnglePos);
          } else if (n == -1) {
            multiplaneServo.write(neutralAngle);
          }
          multiPlaneMark = multiPlaneMark + 1;
          delay(200);
          radio.flush_rx();
        }
      } else if(joyPosArray[5] == -1) {
        if (multiPlaneMark != -1) {
          if (multiPlaneMark ==0) {
            multiplaneServo.write(critAngleNeg);
          } else if (n == 1) {
            multiplaneServo.write(neutralAngle);
          }
          multiPlaneMark = multiPlaneMark - 1;
          delay(200);
          radio.flush_rx();
        }
      }
      //look in transmitter code for explanation of this element
      else if (joyPosArray[5] == 2) {
        Serial.println("**************************");
        Serial.println(prevPos[0]);
        Serial.println(prevPos[1]);
        Serial.println(prevPos[2]);
        Serial.println(prevPos[3]);
        Serial.println("**************************");
        for (int i = 0; i < 4; i++) {
          savedPos[i] = prevPos[i];
        }
      } else if (joyPosArray[5] == 3) {
        Serial.println("**************************");
        Serial.println(savedPos[0]);
        Serial.println(savedPos[1]);
        Serial.println(savedPos[2]);
        Serial.println(savedPos[3]);
        Serial.println("**************************");
        servoTop.write(savedPos[0]);
        servoTopPos = savedPos[0];
        radio.flush_rx();
        servoBot.write(savedPos[1]);
        servoBotPos = savedPos[1];
        radio.flush_rx();
        float stepperDiff = stepperArray[0].currentAngle - savedPos[2];
        if (stepperDiff < 0) {
          Stepper::incrementalPosMulti(abs(stepperDiff),1,stepperArray); // synchronous stepper movement direction 0 (CCW)
          radio.flush_rx();
        } else {
          Stepper::incrementalPosMulti(abs(stepperDiff),0,stepperArray); // synchronous stepper movement direction 0 (CCW)
          radio.flush_rx();
        }
        float linDiff = linearActuator.position - savedPos[3];
        if (linDiff < 0) {
          linearActuator.incrementalPos(abs(linDiff), 1);
          radio.flush_rx();
        } else {
          linearActuator.incrementalPos(abs(linDiff), 0);
          radio.flush_rx();
        }
      }
    }
    radio.flush_rx();
    radio.stopListening();
    //Serial.println("stopped listening");
    delay(5);
    posMetrics[0] = servoTopPos;
    posMetrics[1] = servoBotPos;
    posMetrics[2] = stepperArray[0].currentAngle;
    posMetrics[3] = linearActuator.position;
    radio.write(&posMetrics, sizeof(posMetrics));
  }
}
