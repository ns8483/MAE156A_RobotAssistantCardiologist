// Rotation
int EN_TIP_PIN = 10; // Enable
int STEP_TIP_PIN = 9; // Step
int DIR_TIP_PIN = 8; // Direction
int tipGearRatio = 40; //40:1 ratio
int EN_HANDLE_PIN = 7; // Enable
int STEP_HANDLE_PIN = 6; // Step
int DIR_HANDLE_PIN = 5; // Direction
double handleGearRatio = 6.67; //6.67:1 ratio

// Linear
int enA = 11; // Motor contoller voltage control (PWM)
int in1 = 12; // Motor direction
int in2 = 13; // Motor direction
int encoderPinA = 2; // Hall Sensor digital signal 
int encoderPinB = 3; // 90 deg phase difference
int power = 255; // PWM power sent to motor controller (0-255 min-max)
float position = 0; // actuator position [mm]
int rotDir = 0; // 0 = retracting, 1 = extending)
float pulseFraction = (float) 1/45; // 45 pulses per mm of extension
volatile boolean A_set = false;
volatile boolean B_set = false;

int customDelay, customDelayMapped;

// Define the number of steps per revolution for your stepper motor
const int microSteps = 8;
const int stepsPerRevolution = 200*microSteps;  // Change this according to your motor's specification
// Keep track of angle
double currentAngle = 0.00;
const int wormGearRatio = 40; //40:1 ratio
int stepperDir = 0; // 0 CW ; 1 CCW

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // allow printing to command line
  // Translation
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  // Check for changes on feedback pin
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);
  // Wait for things to load
  Serial.println("Booting...\n");
  delay(5000);
  Serial.println("Sending to Origin...\n");
  setupActuator();
}

void loop() {
  // Prompt the user for input:
  Serial.println(String("Please enter an option number: \n") + String("1. Linear Translation \n") + String("2. Rotation \n"));
  while (!Serial.available()) {} // Wait for input
  int userInput = Serial.parseInt(); // Read the input from the serial monitor:
  Serial.read();// clear buffer
  switch (userInput) {
    case 1:
      Serial.println(String("Please enter an option number: \n") + String("1. Incremental Steps\n") + String("2. Absolute Position \n") + String("3. Back \n"));
      while (!Serial.available()) {}// Wait for input
      userInput = Serial.parseInt(); // Read the input from the serial monitor:
      Serial.read();// clear buffer
      switch (userInput) {
        case 1:
          linearSteps();
          break;
        case 2:
          linearAbs();
          break;
        case 3:
          break;
      }
      break;
    case 2:
      Serial.println(String("Please enter an option number: \n") + String("1. Incremental Steps\n") + String("2. Back \n"));
      while (!Serial.available()) {}// Wait for input
      userInput = Serial.parseInt(); // Read the input from the serial monitor:
      Serial.read();// clear buffer
      switch (userInput) {
        case 1:
          rotationScript();
          break;
        case 2:
          break;
      }
      break;
    }
}

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
    static unsigned long currentMillis; // Static member variable to hold the current millis value
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


Stepper stepperTip(EN_TIP_PIN, STEP_TIP_PIN, DIR_TIP_PIN, 8, tipGearRatio);
Stepper stepperHandle (EN_HANDLE_PIN, STEP_HANDLE_PIN, DIR_HANDLE_PIN, 8, handleGearRatio);
Stepper stepperArray[] = {stepperTip, stepperHandle};

void rotationScript(){
  float stepSize = 1.0f;
  // Prompt the user for input:
  Serial.println(String("Choose a step size: \n") + String("1. 0.1 deg \n") + String("2. 1.0 deg \n") + String("3. 10.0 deg \n"));
  while (!Serial.available()) {} // Wait for input
  int userInput = Serial.parseInt(); // Read the input from the serial monitor:
  Serial.read();// clear buffer
  switch (userInput) {
    case 1:
      stepSize = 0.1f;
      break;
    case 2:
      stepSize = 1.0f;
      break;
    case 3:
      stepSize = 10.0f;
      break;
  }
  // Prompt the user for input:
  Serial.println(String("Current Step Size: ") + String(stepSize) + " deg \n" + String("D key: Rotate CW \n") + String("A key: Rotate CCW \n"));
  while (true) {
    if (Serial.available()) {
      char inChar = Serial.read();  //read the character
      if (inChar == 'd'){
        Stepper::incrementalPosMulti(stepSize,1,stepperArray);
      } else if (inChar == 'a'){
        Stepper::incrementalPosMulti(stepSize,0,stepperArray);
      }
    }
  }
} 

void linearSpeedControl() {
  power = analogRead(A1); // Read potentiometer
  power = map(power, 0, 1023, 0, 255); // Convert the analog input from 0 to 1024, to 300 to 3000
}

void updateEncoder() {
  static boolean A_prev = false;
  static boolean B_prev = false;
  
  boolean A_current = digitalRead(encoderPinA) == HIGH;
  boolean B_current = digitalRead(encoderPinB) == HIGH;

  if (A_prev && !A_current && B_prev && B_current) {
    // CW transition: A goes LOW before B
    rotDir = 0; // retracting
    position = position - pulseFraction;
  } else if (A_prev && A_current && B_prev && !B_current) {
    // CCW transition: B goes LOW before A
    rotDir = 1; // extending
    position = position + pulseFraction;
  }

  A_prev = A_current;
  B_prev = B_current;
  //Serial.println(String(A_prev) + String(B_prev));
  Serial.println("Current Position: " + String(position) + " mm\n");
}

void changeDirection(int chosenDirection) {
  if (chosenDirection == 1) {
        digitalWrite(in1, HIGH); // extend
        digitalWrite(in2, LOW);
        analogWrite(enA, 255);
        rotDir = 1;
  } else if (chosenDirection == 0) {
        digitalWrite(in1, LOW); // retarct
        digitalWrite(in2, HIGH);
        analogWrite(enA, 255);
        rotDir = 0;
  }
}

void stopActuator() {
  digitalWrite(in1, LOW); // Stop
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
}

bool finishedStep(float stepSize) {
      if(rotDir == 0){stepSize = -1*stepSize;} // make stepSize negative if retarcting
      static bool hasFinished = false;
      static float oldPosition = position;
      static float targetPosition = oldPosition + stepSize; 
      // update the statics with new values upon successful prior completion
      if(hasFinished) {
        oldPosition = position;
        targetPosition = oldPosition + stepSize;
        hasFinished = false;
      }
      //Serial.println("\n Target: " + String(targetPosition) + "\n Old: " + String(oldPosition) + "\n Current: " + String(position) + "\n Step: " + String(stepSize));
      // check to see if target has been met
      if(rotDir == 1) {
        if(position >= targetPosition){
          hasFinished = true; 
          return 1; // actuator has finished step
        } else {
          return 0; // actuator still extending
      }
      } else if(rotDir == 0) {
        if(position <= targetPosition){
          hasFinished = true; 
          return 1; // actuator has finished step
        } else {
          return 0; // actuator still retracting
        }
      }
}

void linearSteps() {
  float stepSize = 1.0f;
  // Prompt the user for input:
  Serial.println(String("Choose a step size: \n") + String("1. 0.1 mm \n") + String("2. 1.0 mm \n") + String("3. 10.0 mm \n"));
  while (!Serial.available()) {} // Wait for input
  int userInput = Serial.parseInt(); // Read the input from the serial monitor:
  Serial.read();// clear buffer
  switch (userInput) {
    case 1:
      stepSize = 0.1f;
      break;
    case 2:
      stepSize = 1.0f;
      break;
    case 3:
      stepSize = 10.0f;
      break;
  }
  // Prompt the user for input:
  Serial.println(String("Current Step Size: ") + String(stepSize) + " mm \n" + String("W key: Step Forward\n") + String("S key: Step Back\n"));
  while (1==1) {
    if (Serial.available()) {
      char inChar = Serial.read();  //read the character
      if (inChar == 'w'){
        changeDirection(1);
        while(!finishedStep(stepSize)) {} // wait for step completion
        stopActuator();
      } else if (inChar == 's'){
        changeDirection(0);
        while(!finishedStep(stepSize)) {} // wait for step completion
        stopActuator();
      }
    }
  } // Wait for input
}

void linearAbs() {

}

void setupActuator(){
      // detract fully
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, 255);
      delay(300); // wait for actuator to start moving
      while (!isZerod(position)) {
        //wait until actuator is zerod
        delay(50);
      }
      // zero constants 
      position = 0;
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      delay(100);
      Serial.println("Ready to Use! \n");
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

  
