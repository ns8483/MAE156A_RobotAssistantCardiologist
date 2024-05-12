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
Servo servoTop; // top servo object creation
Servo servoBot; // bottom servo object creation
// NRF Transceiver
int NRF_CSN_PIN = 8;
int NRF_CE_PIN = 7;
int NRF_SCK_PIN = 52;
int NRF_MOSI_PIN = 51;
int NRF_MISO_PIN = 50;
const byte address[6] = "00001";
RF24 radio(NRF_CE_PIN,NRF_CSN_PIN); // radio object creation
// Joystick tolerances after mapping
int UPLIM = 80;
int DOWNLIM = -80;
int RLIM = 80;
int LLIM = -80;
// Global Vars
float linearStepSize = 1.00; // default linear step size [mm]
float rotationStepSize = 1.00; // default rotation step size [deg]
float servoStepSize = 1.00; // default knob rotation step size [deg]
int microSteps = 8; // default number of stepper microsteps
bool busy = false; // true if the system is doing task

void setup() {
  Serial.begin(115200);
  Serial.println("Booting...\n");
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
  radio.openReadingPipe(0, address); // 00001
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop(){
  // initialize class objects
  Stepper stepperTip(EN_TIP_PIN, STEP_TIP_PIN, DIR_TIP_PIN, microSteps, tipGearRatio);
  Stepper stepperHandle (EN_HANDLE_PIN, STEP_HANDLE_PIN, DIR_HANDLE_PIN, microSteps, handleGearRatio);
  Stepper stepperArray[] = {stepperTip, stepperHandle};
  LinearActuator linearActuator(ENA_PIN_NUM, IN1_PIN_NUM, IN2_PIN_NUM, ENCODERA_PIN_NUM, ENCODERB_PIN_NUM);
  while(true) {
    Serial.println(String("connecting..."));
    if (radio.available()) {
      while(radio.available() && !busy) {
        int joyPosArray[4] = {};
        radio.read(&joyPosArray, sizeof(joyPosArray));
        Serial.println(String("x1: ") + String(joyPosArray[0]) + String("  y1: ") + String(joyPosArray[1]) + String("  x2: ") + String(joyPosArray[2]) + String("  y2: ") + String(joyPosArray[3]));
        if (joyPosArray[0] <= LLIM){
          busy = true;
          if (servoTopPos != 0){
            Serial.println("left");
            servoTopPos = servoTopPos - servoStepSize;
            servoTop.write(servoTopPos);
            Serial.println(servoTopPos);
          }
          busy = false;
        } else if (joyPosArray[0] >= RLIM){
          busy = true;
          if (servoTopPos != 180){
            Serial.println(String(servoTopPos));
            servoTopPos = servoTopPos + servoStepSize;
            servoTop.write(servoTopPos);
            Serial.println(servoTopPos);
          }
          busy = false;
        } else if (joyPosArray[1] <= DOWNLIM){
          busy = true;
          if (servoBotPos != 0){
            servoBotPos = servoBotPos - servoStepSize;
            servoBot.write(servoBotPos);
            Serial.println(servoBotPos);
          }
          busy = false;
        } else if (joyPosArray[1] >= UPLIM){
            busy = true;
            if (servoBotPos != 180){
              servoBotPos = servoBotPos + servoStepSize;
              servoBot.write(servoBotPos);
              Serial.println(servoBotPos);
            }
            busy = false;
        } else if (joyPosArray[2] >= RLIM){
          busy = true;
          Stepper::incrementalPosMulti(rotationStepSize,0,stepperArray); // synchronous stepper movement direction 1 (CW)
          busy = false;
        } else if (joyPosArray[2] <= LLIM){
          busy = true;
          Stepper::incrementalPosMulti(rotationStepSize,1,stepperArray); // synchronous stepper movement direction 0 (CCW)
          busy = false;
        }else if (joyPosArray[3] >= UPLIM){
          busy = true;
          Serial.println("hey");
          linearActuator.incrementalPos(linearStepSize, 1); // linear actuator extension
          busy = false;
        }else if (joyPosArray[3] <= DOWNLIM){
          linearActuator.incrementalPos(linearStepSize, 0); // linear actuator retraction
          busy = false;
        }
        delay(200);
      }
    }
  }
}