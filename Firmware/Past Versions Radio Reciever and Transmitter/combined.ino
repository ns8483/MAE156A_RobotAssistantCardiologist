// Libraries
#include <Servo.h>
#include <Stepper.h>
#include <LinearActuator.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
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
const byte address[6] = "00001"; 

void setup() {
  Serial.begin(115200); // open serial
  Serial.println("Booting...\n");
  loop();
}

void loop(){
  // initialize class objects
  Stepper stepperTip(EN_TIP_PIN, STEP_TIP_PIN, DIR_TIP_PIN, microSteps, tipGearRatio);
  Stepper stepperHandle (EN_HANDLE_PIN, STEP_HANDLE_PIN, DIR_HANDLE_PIN, microSteps, handleGearRatio);
  Stepper stepperArray[] = {stepperTip, stepperHandle};
  LinearActuator linearActuator(ENA_PIN_NUM, IN1_PIN_NUM, IN2_PIN_NUM, ENCODERA_PIN_NUM, ENCODERB_PIN_NUM);
  Servo servoTop;
  Servo servoBot;
  RF24 radio(NRF_CE_PIN,NRF_CSN_PIN);
  //radio setup
  
  // servo setup
  servoTop.attach(SERVO_TOP_PIN); // attach feedback pins to servo objects
  servoBot.attach(SERVO_BOT_PIN); 
  servoTop.write(servoTopPos); // send servos to home
  servoBot.write(servoBotPos);
  // main keyboard interface
  Serial.println("Welcome to UCSD Team 24 Cardiology Robot");
  Serial.println(String("Please enter an option number: \n") + String("1. Wired Keyboard Controller \n") + String("2. Wireless Controller\n") + String("3. Settings\n"));
  while (!Serial.available()) {} // Wait for input
  int userInput = Serial.parseInt(); // Read the input from the serial monitor:
  Serial.read(); // clear buffer
  switch (userInput) {
    case 1:
    while(true) {
        Serial.println("Wired Keyboard Controller \n");
        Serial.println(String("Current positioning:"));
        Serial.println(String("Linear Translation: ") + String(linearActuator.position) + String(" mm   W key (extend) S key (retract)"));
        Serial.println(String("Rotation: 0.00°   D key (CW) A key (CCW)"));
        Serial.println(String("Left/Right flexion: 0.00°   J key (left) L key (right)"));
        Serial.println(String("Anteflex/Retroflex: 0.00°   I key (ante) K key (retro)"));
        Serial.println(String("X key - Go Back\n"));
        while (!Serial.available()) {};// Wait for input
        char command = Serial.read(); // Read the character from the serial monitor:
        if (command == 'w'){
          linearActuator.incrementalPos(linearStepSize, 1); // linear actuator extension
        } else if (command == 's'){
          linearActuator.incrementalPos(linearStepSize, 0); // linear actuator retraction
        }else if (command == 'd'){
          Stepper::incrementalPosMulti(rotationStepSize,0,stepperArray); // synchronous stepper movement direction 1 (CW)
        }else if (command == 'a'){
          Stepper::incrementalPosMulti(rotationStepSize,1,stepperArray); // synchronous stepper movement direction 0 (CCW)
        }else if (command == 'j'){
          if (servoTopPos != 180){
            servoTopPos = servoTopPos + servoStepSize;
            servoTop.write(servoTopPos);
            Serial.println(servoTopPos);
          }
        }else if (command == 'l'){
          if (servoTopPos != 0){
            servoTopPos = servoTopPos - servoStepSize;
            servoTop.write(servoTopPos);
            Serial.println(servoTopPos);
          }
        }else if (command == 'i'){
          if (servoBotPos != 180){
            servoTopPos = servoBotPos + servoStepSize;
            servoBot.write(servoBotPos);
            Serial.println(servoBotPos);
          }
        }else if (command == 'k'){
          if (servoBotPos != 0){
            servoBotPos = servoBotPos - servoStepSize;
            servoBot.write(servoBotPos);
            Serial.println(servoBotPos);
          }
        }else if (command == 'x'){
          break;
        }
        Serial.read(); // clear buffer
      }
    case 2:
      Serial.println("Wireless Controller \n");
      radio.startListening();
      while(true){
        if (radio.available()) {
          while (radio.available()){
            int angleV = 0;
            radio.read(&angleV,sizeof(angleV));
            Serial.println(String(angleV));
          }
          delay(5);
          radio.stopListening();
          char text[32] = "";
          radio.read(&text, sizeof(text));
        }
        Serial.println("ye");
      }
    case 3:
      break;
  }
}

class myServo {
  private:
    int pos = 0;
    int feedbackPin;
    Servo s;
  public:
    myServo(int FEEDBACK_PIN){
      feedbackPin = FEEDBACK_PIN;
    }
    void calibrate_servo(Servo s, int pos, int pin) {
      s.attach(pin);
      int i = 0;
      for (i=0; i<180; i++){
        s.write(i);
        delay(100);
      }
      s.write(90);
      delay(1000);
    }
};

