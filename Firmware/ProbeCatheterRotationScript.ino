// Libraries
#include <Stepper.h>

// Rotation
int EN_PIN = 7; // Enable
int DIR_PIN = 10; // Motor contoller voltage control (PWM)
int STEP_PIN = 4; // Motor direction

// Linear
int enA = 9; // Motor contoller voltage control (PWM)
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
  // Rotation
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(EN_PIN,LOW); // Enable TMC2209
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
          rotationSteps();
          break;
        case 2:
        
          break;
      }
      break;
    }
  //pulses with custom delay
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(customDelayMapped);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(customDelayMapped);
  analogWrite(enA, power);
  
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

void changeStepperDirection(int chosenDirection) {
  if (chosenDirection == 0) { // CW 
        digitalWrite(DIR_PIN, LOW);
        stepperDir = 0;
  } else if (chosenDirection == 1) { // CCW 
        digitalWrite(DIR_PIN, HIGH);
        stepperDir = 1;
  }
}

void singleStep(){
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(3000);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(500);
  if (stepperDir == 0){
    currentAngle = currentAngle - 360.00/(static_cast<double>(stepsPerRevolution)*static_cast<double>(wormGearRatio)); // negative for CW
  } else{
    currentAngle = currentAngle + 360.00/(static_cast<double>(stepsPerRevolution)*static_cast<double>(wormGearRatio)); // update current angle
  }
  Serial.println("\nCurrent Angle: " + String(currentAngle) + " deg");
}

void stopActuator() {
  digitalWrite(in1, LOW); // Stop
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
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

void rotationSteps(){
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
  while (1==1) {
    if (Serial.available()) {
      char inChar = Serial.read();  //read the character
      if (inChar == 'd'){
        changeStepperDirection(0); // CW
        while(!reachedStepperTarget(stepSize)) {
          singleStep();
        }
      } else if (inChar == 'a'){
        changeStepperDirection(1); // CCW
        while(!reachedStepperTarget(stepSize)) {
          singleStep();
        }
      }
    }
  } // Wait for input
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

bool reachedStepperTarget(float stepSize) {
  static bool hasFinished = false;
  static int steps = 0;
  static float targetSteps = (stepSize/360)*stepsPerRevolution*wormGearRatio;
  // update the statics with new values upon successful prior completion
  if(hasFinished) {
    steps = 0;
    targetSteps = (stepSize/360)*stepsPerRevolution*wormGearRatio;
    hasFinished = false;
  }
  //Serial.println("\n steps: " + String(steps) + "\ntargetSteps: " + String(targetSteps));
  if (steps < targetSteps){
    steps ++; //add a step
    return 0; // not finsihed rev
  } else if (steps >= targetSteps){
    hasFinished = true;
    return 1; // finsihed rev
  }
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
