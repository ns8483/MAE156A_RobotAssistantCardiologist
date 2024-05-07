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
    static LinearActuator* instancePointer; // instance pointer allows indirect access of non-static vars from static methods
    
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // allow printing to command line
  // wait for things to load
  Serial.println("Booting...\n");
  delay(500);
  loop(); // run main loop
}


void loop(){
  // initialize class objects
  Stepper stepperTip(EN_TIP_PIN, STEP_TIP_PIN, DIR_TIP_PIN, microSteps, tipGearRatio);
  Stepper stepperHandle (EN_HANDLE_PIN, STEP_HANDLE_PIN, DIR_HANDLE_PIN, microSteps, handleGearRatio);
  Stepper stepperArray[] = {stepperTip, stepperHandle};
  LinearActuator linearActuator(ENA_PIN_NUM, IN1_PIN_NUM, IN2_PIN_NUM, ENCODERA_PIN_NUM, ENCODERB_PIN_NUM);
  Serial.println("Welcome to UCSD Team 24 Cardiology Robot");
  Serial.println(String("Please enter an option number: \n") + String("1. Wired Keyboard Controller \n") + String("2. Wireless Controller\n") + String("3. Settings"));
  while (!Serial.available()) {} // Wait for input
  int userInput = Serial.parseInt(); // Read the input from the serial monitor:
  Serial.read();// clear buffer
  switch (userInput) {
    case 1:
      Serial.println("Wired Keyboard Controller \n");
      Serial.println(String("Current positioning: \n") + String("Linear Translation: 0.00 mm \n") + String("Rotation: 0.00 deg\n"));
      Serial.println(String("Keyboard Controls: \n") + String("W key: extend \n") + String("S key: retract") + String("D key: rotate CW") + String("A key: rotate CCW"));
      while (!Serial.available()) {};// Wait for input
      char inChar = Serial.read(); // Read the character from the serial monitor:
      if (inChar == 'w'){
        linearActuator.incrementalPos(linearStepSize, 1); // linear actuator extension
        break;
      } else if (inChar == 's'){
        linearActuator.incrementalPos(linearStepSize, 0); // linear actuator retraction
        break;
      }else if (inChar == 'd'){
        Stepper::incrementalPosMulti(rotationStepSize,1,stepperArray); // synchronous stepper movement direction 1 (CW)
        break;
      }else if (inChar == 'a'){
        Stepper::incrementalPosMulti(rotationStepSize,0,stepperArray); // synchronous stepper movement direction 0 (CCW)
        break;
      }
    case 2:
      break;
    case 3:
      break;
  }
}



  
