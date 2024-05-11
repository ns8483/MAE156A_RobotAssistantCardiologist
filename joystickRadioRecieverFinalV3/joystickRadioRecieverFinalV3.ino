// Libraries
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "newLinearActuator.h"
#include "newStepper.h"

//class variable definitions
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
int servoBotPos = 90; // default position
// NRF Transceiver
int NRF_CSN_PIN = 8;
int NRF_CE_PIN = 7;
int NRF_SCK_PIN = 52;
int NRF_MOSI_PIN = 51;
int NRF_MISO_PIN = 50;
// Global Vars
float linearStepSize = 5.00; // default linear step size [mm]
float rotationStepSize = 5.00; // default rotation step size [deg]
float servoStepSize = 5.00; // default knob rotation step size [deg]
int microSteps = 8; // default number of stepper microsteps

// Radio initialized
#define CE_PIN 7;
#define CSN_PIN 8;
RF24 radio(CE_PIN,CSN_PIN);
const byte address[6] = "00001";

// initialize class objects
Stepper stepperTip(EN_TIP_PIN, STEP_TIP_PIN, DIR_TIP_PIN, microSteps, tipGearRatio);
Stepper stepperHandle (EN_HANDLE_PIN, STEP_HANDLE_PIN, DIR_HANDLE_PIN, microSteps, handleGearRatio);
Stepper stepperArray[] = {stepperTip, stepperHandle};
LinearActuator linearActuator(ENA_PIN_NUM, IN1_PIN_NUM, IN2_PIN_NUM, ENCODERA_PIN_NUM, ENCODERB_PIN_NUM);
Servo servoTop;
Servo servoBot;

//tested joystick and got these values: ymax: 640, ymin: 0, xmin: 0, xmax: 1023 and added about 20 tolerance in each direction.
int UPLIM = 620;
int DOWNLIM = 15;
int RLIM = 1000;
int LLIM = 20;

// Definitions of static member variables
LinearActuator* LinearActuator::instancePointer = nullptr;

void setup() {
  Serial.begin(9600);
  // servo setup
  servoTop.write(servoTopPos); // send servos to home right away so that will do when attached faster
  servoBot.write(servoBotPos);
  servoTop.attach(SERVO_TOP_PIN); // attach feedback pins to servo objects
  servoBot.attach(SERVO_BOT_PIN); 
  radio.begin();
  radio.openReadingPipe(1, address); // 00001
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.println("Booting...\n");
}

void loop(){
  if (radio.available()) {
    while(radio.available()) {
      int joyPos[4];
      radio.read(&joyPos, sizeof(joyPos));
      //Control of linear actuator extension and retraction, and stepper motor rotation
      if (joyPos[2] >= RLIM ){
        linearActuator.incrementalPos(linearStepSize, 1); // linear actuator extension
      } else if (joyPos[2] <= LLIM){
        linearActuator.incrementalPos(linearStepSize, 0); // linear actuator retraction
      }else if (joyPos[3] >= UPLIM){
        Stepper::incrementalPosMulti(rotationStepSize,0,stepperArray); // synchronous stepper movement direction 1 (CW)
      }else if (joyPos[3] <= DOWNLIM){
        Stepper::incrementalPosMulti(rotationStepSize,1,stepperArray); // synchronous stepper movement direction 0 (CCW)
      }
      //Control of knobs with servo motors
      if (joyPos[1] >= UPLIM){
        if (servoTopPos != 180){
          servoTopPos = servoTopPos + servoStepSize;
          servoTop.write(servoTopPos);
          Serial.println(servoTopPos);
        }
      }else if (joyPos[1] <= DOWNLIM){
        if (servoTopPos != 0){
          servoTopPos = servoTopPos - servoStepSize;
          servoTop.write(servoTopPos);
          Serial.println(servoTopPos);
        }
      }else if (joyPos[0] >= RLIM){
        if (servoBotPos != 180){
          servoTopPos = servoBotPos + servoStepSize;
          servoBot.write(servoBotPos);
          Serial.println(servoBotPos);
        }
      }else if (joyPos[0] <= LLIM){
        if (servoBotPos != 0){
          servoBotPos = servoBotPos - servoStepSize;
          servoBot.write(servoBotPos);
          Serial.println(servoBotPos);
        }
      }
      Serial.read(); // clear buffer
    }
    radio.stopListening();
  }
}