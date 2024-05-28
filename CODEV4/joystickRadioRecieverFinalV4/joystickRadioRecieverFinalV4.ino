// Libraries
#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LinearActuator.h>
#include <Stepper.h>
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
float stepperPrevPos = 0;
// Linear Actuator
int ENA_PIN_NUM = 24; // Motor contoller voltage control (PWM) (change)
int IN1_PIN_NUM = 26; // Motor direction (change)
int IN2_PIN_NUM = 28; // Motor direction (change)
int ENCODERA_PIN_NUM = 3; // Hall Sensor digital signal 
int ENCODERB_PIN_NUM = 2; // 90 deg phase difference
float linPrevPos = 0;
// Knob Rotation
int SERVO_TOP_PIN = 12; // servo for top knob (change)
int SERVO_BOT_PIN = 11; // servo for bottom knob (change)
int servoTopPos = 90; // default position
int prevServoTopPos = servoTopPos; // keeps track of previous servo position
int servoBotPos = 90; // default position
int prevServoBotPos = servoBotPos; // keeps track of previous servo position
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
RF24 radio(NRF_CE_PIN,NRF_CSN_PIN); // radio object creation

//keeps track of what was changed so that can keep track for backtracking.
  //0 means no change, 1 means servo top, 2 means servo bottom, 3 means stepper, 4 means linear actuator 
int manipulated = 0;

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
  // radio setup
  radio.begin();
  radio.openReadingPipe(1, addresses[1]); // 00002
  radio.openWritingPipe(addresses[0]);
  radio.setPALevel(RF24_PA_MIN);
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
      radio.read(&joyPosArray, sizeof(joyPosArray));
      delay(5);
      //Serial.println(String("x1: ") + String(joyPosArray[0]) + String("  y1: ") + String(joyPosArray[1]) + String("  x2: ") + String(joyPosArray[2]) + String("  y2: ") + String(joyPosArray[3]));
      if (joyPosArray[0] <= -1*joyCritical){
        if (servoTopPos != 0){
          prevServoTopPos = servoTopPos;
          servoTopPos = servoTopPos - servoStepSize;
          servoTop.write(servoTopPos);
          delay(200); //wait for servo to reach this position
          radio.flush_rx();
          manipulated = 1;
        }
      }else if (joyPosArray[0] >= joyCritical){
        if (servoTopPos != 180){
          prevServoTopPos = servoTopPos;
          servoTopPos = servoTopPos + servoStepSize;
          servoTop.write(servoTopPos);
          delay(200); //wait for servo to reach this position
          radio.flush_rx();
          manipulated = 1;
        }
      }else if (joyPosArray[1] <= -1*joyCritical){
        if (servoBotPos != 0){
          prevServoBotPos = servoBotPos;
          servoBotPos = servoBotPos - servoStepSize;
          servoBot.write(servoBotPos);
          delay(200); //wait for servo to reach this position
          radio.flush_rx();
          manipulated = 2;
        }
      }else if (joyPosArray[1] >= joyCritical){
        if (servoBotPos != 180){
          prevServoBotPos = servoBotPos;
          servoBotPos = servoBotPos + servoStepSize;
          servoBot.write(servoBotPos);
          delay(200); //wait for servo to reach this position
          radio.flush_rx();
          manipulated = 2;
        }
      }else if (joyPosArray[2] >= joyCritical){ // if left joystick points right and stepper is not already moving
        stepperPrevPos = stepperArray[0].currentAngle;
        Stepper::incrementalPosMulti(rotationStepSize,0,stepperArray); // synchronous stepper movement direction 1 (CW)
        radio.flush_rx();
        manipulated = 3;
      }else if (joyPosArray[2] <= -1*joyCritical){ // if left joystick points left and stepper is not already moving
        stepperPrevPos = stepperArray[0].currentAngle;
        Stepper::incrementalPosMulti(rotationStepSize,1,stepperArray); // synchronous stepper movement direction 0 (CCW)
        radio.flush_rx();
        manipulated = 3;
      }else if (joyPosArray[3] >= joyCritical){ // if left joystick points up
        linPrevPos = linearActuator.position;
        linearActuator.incrementalPos(linearStepSize, 1); // linear actuator extension
        radio.flush_rx();
        manipulated = 4;
      }else if (joyPosArray[3] <= -1*joyCritical){ // if left joystick points down
        linPrevPos = linearActuator.position;
        linearActuator.incrementalPos(linearStepSize, 0); // linear actuator retraction
        radio.flush_rx();
        manipulated = 4;
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
      } //need to add code for servo linear actuator
      //look in transmitter code for explanation of this element
      else if (joyPosArray[5] == 2) {
        if (manipulated == 1){
          servoTop.write(prevServoTopPos);
        } else if (manipulated == 2) {
          servoBot.write(prevServoBotPos);
        } else if (manipulated == 3 || manipulated == -3) {
          while(stepperArray[0].currentAngle != stepperPrevPos){
            if (manipulated == 3) {
              Stepper::incrementalPosMulti(rotationStepSize,1,stepperArray);
            } else if (manipulated == -3) {
              Stepper::incrementalPosMulti(rotationStepSize,0,stepperArray);
            }
            delay(200); //wait for servo to reach this position
            radio.flush_rx();
          }
        } else if (manipulated == 4 || manipulated == -4) {
          while(linearActuator.position != linPrevPos){
            if (manipulated == 4) {
              linearActuator.incrementalPos(-0.1, 0); // linear actuator retraction
            } else if (manipulated == -4) {
              linearActuator.incrementalPos(0.1, 0); // linear actuator retraction
            }
            delay(200); //wait for servo to reach this position
            radio.flush_rx();
          }
        }
        delay(200); //wait for servo to reach this position
        radio.flush_rx();
      }
    }
    radio.stopListening();
    delay(5);
    posMetrics[0] = servoTopPos;
    posMetrics[1] = servoBotPos;
    posMetrics[2] = stepperArray[0].currentAngle;
    posMetrics[3] = linearActuator.position;
    radio.write(&posMetrics, sizeof(posMetrics));

  }
}