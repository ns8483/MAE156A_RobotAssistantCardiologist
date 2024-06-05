#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <LiquidCrystal_I2C.h>

//instantiating LCD display
LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x3F for a 16 chars and 2 line display

//Defining and instantiating radio and parameters
int CE_PIN = 9;
int CSN_PIN = 10;
RF24 radio(CE_PIN,CSN_PIN);
const byte addresses[][6] = {"00001","00002"};
String topServo;
String botServo;
String stepper;
String linActuator;

//Defining joystick pins
#define RIGHT_JOY_X A0
#define RIGHT_JOY_Y A1
int RIGHT_SWITCH = 2;
#define LEFT_JOY_X A2
#define LEFT_JOY_Y A3
int LEFT_SWITCH = 3;

int onOffPin = 4; // Pin connected to the onOff switch

void setup() {
  Serial.begin(115200);

  //set up digital pins for joystick switch
  pinMode(RIGHT_SWITCH, OUTPUT);
  pinMode(LEFT_SWITCH, OUTPUT);
  digitalWrite(RIGHT_SWITCH, HIGH);
  digitalWrite(LEFT_SWITCH, HIGH);
  pinMode(onOffPin, INPUT_PULLUP); // onOff switch

  //setting up rado for reading and writing
  radio.begin();
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
  radio.setPALevel(RF24_PA_MAX);
  radio.flush_rx();

  //setting up LCD display
  lcd.init();
  lcd.clear();         
  lcd.backlight();

  //print opening screen messages and wait for connection
  lcd.setCursor(0,0);
  lcd.print("T24 Cardio Robot");
  lcd.setCursor(0,1);
  lcd.print("Waiting for");
  lcd.setCursor(0,2);
  lcd.print("Connection...");
  radio.startListening();
  while(!radio.available()){} // wait until connection is found
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("Connected!");
  delay(1000);
  lcd.clear();
  lcdSetup("R/L Flex: ", "U/D Flex: ", "Rotation: ", "Translation: "); // setup lcd
}

void loop() {
  while (emergencyStop()){} // while emergency stop is true wait
  radio.stopListening();
  //Sends it to radio for communication
  float joyPosArray[6];
  joyPosArray[0]= map(analogRead(RIGHT_JOY_X), 0, 1023, -100, 100);
  joyPosArray[1]= map(analogRead(RIGHT_JOY_Y), 0, 1023, 100, -100);
  joyPosArray[2]= map(analogRead(LEFT_JOY_X), 0, 1023, 100,-100);
  joyPosArray[3]= map(analogRead(LEFT_JOY_Y), 0, 1023, -100,100);
  //for joyPosArray[4], 0 means no change, -1 means tip flextion step decreases, 1 means tip flexion step increases, 
    //-2 means rotation step decreases, 2 means rotation step increases, 
      //-3 means lin actuator step decreases, 3 means lin actuator step increases
  joyPosArray[4] = 0;
  //for joyPosArray[5], 0 means exit menu and no change
    //1 means plane angle increase and -1 means plane angle decrease
      //2 means that previous position is restored
  joyPosArray[5] = 0;

  //when left switch is pressed, there is an option to change the multiplane angle or restore the previous position
  if (digitalRead(LEFT_SWITCH) == 0) {
    //options are printed out
    lcd.clear();
    lcd.setCursor(0,0);
    //set up lcd with menu
    lcdSetup("Exit Menu","Multiplane Angle", "Save Position", "Restore Position");
    delay(100);
    //toggle through menu using left joystick
    int n = selectMenu(LEFT_SWITCH, LEFT_JOY_Y);
    if (n == 0){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("EXITING...");
    }
    //option to increase or decrease multiplane angle
    if (n == 1){
      n = choiceButtons("<- (-)","(+) ->","MOTOR CW","MOTOR CCW",n);
      delay(200);
    }
    //confirmation of selection
    if (n == 2) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("POSITION SAVED");
    }
    //confirmation of selection
    if (n == 3) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("POSITION RESTORED");
    }
    //write data to reciever
    joyPosArray[5] = float(n);
    Serial.println(joyPosArray[5]);
    Serial.println(joyPosArray[6]);
    delay(500);
    lcdSetup("R/L Flex: ", "U/D Flex: ", "Rotation: ", "Translation: "); // re-setup lcd
  }

  //if right button is pressed, menu for step sizes to change appears
  if (digitalRead(RIGHT_SWITCH) == 0) {
    lcd.clear();
    lcdSetup("Exit Menu","Flexion Step", "Rotation Step", "Translation Step");
    delay(100);
    int d = selectMenu(RIGHT_SWITCH, RIGHT_JOY_Y);
    if (d == 0){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("EXITING...");
    }
    if (d > 0){
      d = choiceButtons("<- (-)","(+) ->","STEP DECREASED","STEP INCREASED",d);
      delay(200);
    }
    joyPosArray[4] = float(d);
    Serial.println(d);
    delay(500);
    lcdSetup("R/L Flex: ", "U/D Flex: ", "Rotation: ", "Translation: ");
  }

  //useufl in case of testing purposes
  Serial.println("**************************");
  Serial.println(joyPosArray[0]);
  Serial.println(joyPosArray[1]);
  Serial.println(joyPosArray[2]);
  Serial.println(joyPosArray[3]);
  Serial.println(joyPosArray[4]);
  Serial.println(joyPosArray[5]);
  Serial.println(joyPosArray[6]);
  Serial.println("**************************");

  //Serial.println(joyPosArray[5]);
  radio.write(&joyPosArray, sizeof(joyPosArray));  
  radio.startListening();
  delay(5);
  // MAIN loop
  if (radio.available()) {
    float posMetrics[4];
    //reads position metrics from robot and updates lcd display on controller
    radio.read(&posMetrics, sizeof(posMetrics));
    topServo = String(int(posMetrics[0]));
    botServo = String(int(posMetrics[1]));
    stepper = String(int(posMetrics[2]));
    linActuator = String(posMetrics[3],1);
    for (int i = 0; i < 3; i++){
      if (posMetrics[i]<10 || (-100<posMetrics[i] && posMetrics[i]<0)){
        lcd.setCursor(11, i);
        lcd.print("    ");
      }
      else if(posMetrics[i]<100 || (-10<posMetrics[i] && posMetrics[i]<0) ){
        lcd.setCursor(12, i);
        lcd.print("    ");
      }
    }
    if (posMetrics[3]<10){
      lcd.setCursor(19, 3);
      lcd.print(" ");  
    }
    lcd.setCursor(10, 0);
    lcd.print(topServo + char(223));
    //Serial.println("top servo: " + topServo);
    lcd.setCursor(10, 1);
    lcd.print(botServo + char(223));
    //Serial.println("bot servo: " + botServo);
    lcd.setCursor(10, 2);
    lcd.print(stepper + char(223));
    //Serial.println("stepper: " + stepper);
    lcd.setCursor(13, 3);
    lcd.print(linActuator + " mm");
    //Serial.println("lin actuator: " + linActuator);
  }
}

//emergency stop switch code, just pauses entire process
bool emergencyStop(){
  static bool switched = true;
  if (digitalRead(onOffPin) == LOW){ // Check if onOff switch is pressed
    if (switched){
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("Emergency Stop");
      lcd.setCursor(0, 2);
      lcd.print("Activated"); 
      switched = false;
    }
    return 1;
  } else if (digitalRead(onOffPin) == HIGH){
    if (!switched){lcdSetup("R/L Flex: ", "U/D Flex: ", "Rotation: ", "Translation: ");} // re-setup lcd
    switched = true;
    return 0;
  }
}

//sets up screen of lcd with list
void lcdSetup(String s1, String s2, String s3, String s4){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(s1);
  lcd.setCursor(0, 1);
  lcd.print(s2);
  lcd.setCursor(0, 2);
  lcd.print(s3);
  lcd.setCursor(0, 3);
  lcd.print(s4);
}

// toggles through list menu and allows for making of selection
int selectMenu(int button, int analogPin) {
  int downCount = 0;
  int joy = 0;
  int joy_max_crit = 100;
  int joy_min_crit = -100;
  if (analogPin == RIGHT_JOY_Y) {
    joy = RIGHT_JOY_Y;
  } else {
    joy = LEFT_JOY_Y;
    joy_max_crit = -100;
    joy_min_crit = 100;
  }
  while(digitalRead(button) == 1) {
    if (map(analogRead(joy), 0, 1023, joy_min_crit, joy_max_crit) == 100) {
      lcd.setCursor(19, downCount);
      lcd.print(" ");
      if (downCount != 3) {
        downCount = downCount + 1;
      } else {
        downCount = 0;
      }
    }
    else if (map(analogRead(joy), 0, 1023, joy_min_crit, joy_max_crit) == -100){
      lcd.setCursor(19, downCount);
      lcd.print(" ");
      if (downCount != 0) {
      downCount = downCount - 1;
      } else {
        downCount = 3;
      }
    }
    Serial.println(downCount);
    lcd.setCursor(19, downCount%4);
    lcd.print("*");
    delay(200);
  }
  return downCount;
}

//can press one of two buttons to choose between choices (usually increase or decrease)
int choiceButtons(String choice1, String choice2, String affirm1, String affirm2, int count) {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(choice1);
  lcd.setCursor(13,0);
  lcd.print(choice2);
  delay(300);
  while(true){
    if (digitalRead(RIGHT_SWITCH) == 0) {
      lcd.clear();
      lcd.print(affirm2);
      break;
    }
    if (digitalRead(LEFT_SWITCH)==0) {
      lcd.clear();
      lcd.print(affirm1);
      count = count * -1;
      break;
    }
  }
  return count;
}