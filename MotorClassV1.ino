#include <Stepper.h>

class NewStepper{
  //class variables
  int EN_PIN;
  int DIR_PIN;
  int STEP_PIN;
  int MICRO_STEPS;
  int STEPS_PER_REV;
  int STEPPER_DIR; //will be 0 for CW and 1 for CCW
  double currentAngle;

  public:
  NewStepper(int en_pin,int dir_pin,int step_pin, int microSteps){
    EN_PIN = en_pin; //enable
    DIR_PIN = dir_pin; //motor controller voltage control
    STEP_PIN = step_pin; // motor direction
    MICRO_STEPS = microSteps; 
    STEPS_PER_REV = MICRO_STEPS*200; //will be a function of microSteps depending on motor specs
    STEPPER_DIR = 0;
    currentAngle = 0;

    pinMode(EN_PIN,OUTPUT);
    pinMode(DIR_PIN,OUTPUT);
    pinMode(STEP_PIN,OUTPUT);
    digitalWrite(EN_PIN,LOW);
  }

  void changeStepperDirection(int chosenDirection) {
    if (chosenDirection == 0) { // CW 
      digitalWrite(DIR_PIN, LOW);
      STEPPER_DIR = 0;
    }
    else if (chosenDirection == 1) { // CCW 
      digitalWrite(DIR_PIN, HIGH);
      STEPPER_DIR = 1;
    }
  }

  void singleStep(){
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(3000);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500);
    if (STEPPER_DIR == 0){
      currentAngle = currentAngle - 360.00/(static_cast<double>(STEPS_PER_REV)); // negative for CW
    } 
    else {
      currentAngle = currentAngle + 360.00/(static_cast<double>(STEPS_PER_REV)); // update current angle
    }
  }
};

//only contains the class, will work on the rest after bluetooth integration
void setup(){

}

void loop() {

}
