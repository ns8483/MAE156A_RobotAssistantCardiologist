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

//joystick limit parameters
int LR_MIN = -100;
int LR_MAX = 100;
int UD_MIN = -100;
int UD_MAX = 100;


void setup() {
  Serial.begin(115200);

  //set up digital pins for joystick switch
  pinMode(RIGHT_SWITCH, OUTPUT);
  pinMode(LEFT_SWITCH, OUTPUT);
  digitalWrite(RIGHT_SWITCH, HIGH);
  digitalWrite(LEFT_SWITCH, HIGH);

  radio.begin();
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
  radio.setPALevel(RF24_PA_MIN);

  //setting up LCD display
  lcd.init();
  lcd.clear();         
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("T24 Cardio Robot");
  lcd.setCursor(0,1);
  lcd.print("Connecting...");
  delay(700);
  lcd.setCursor(0,2);
  lcd.print("Connected!");
  delay(500);
  lcd.clear();
  
  lcd.setCursor(0, 0);
  lcd.print("R/L Flex: ");
  lcd.setCursor(0, 1);
  lcd.print("U/D Flex: ");
  lcd.setCursor(0, 2);
  lcd.print("Rotation: ");
  lcd.setCursor(0, 3);
  lcd.print("Translation: ");
}

void loop() {
  radio.stopListening();
  delay(20);
  //Sends it to radio for communication
  /*int joyPos[4];
  int joyPosArray[4];*/
  int joyPosArray[5];
  /*joyPos[0]= analogRead(RIGHT_JOY_X);
  joyPos[1]= analogRead(RIGHT_JOY_Y);
  joyPos[2] = analogRead(LEFT_JOY_X);
  joyPos[3] = analogRead(LEFT_JOY_Y);*/
  joyPosArray[0]= map(analogRead(RIGHT_JOY_X), 0, 1023, -100, 100);
  joyPosArray[1]= map(analogRead(RIGHT_JOY_Y), 0, 1023, 100, -100);
  joyPosArray[2]= map(analogRead(LEFT_JOY_X), 0, 1023, 100,-100);
  joyPosArray[3]= map(analogRead(LEFT_JOY_Y), 0, 1023, -100,100);
  //for joyPosArray[4], 0 means no change, -1 means tip flextion step decreases, 1 means tip flexion step increases, 
    //-2 means rotation step decreases, 2 means rotation step increases, 
      //-3 means lin actuator step decreases, 3 means lin actuator step increases
  joyPosArray[4] = 0;
  if (digitalRead(RIGHT_SWITCH) == 0) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Exit Menu");
    lcd.setCursor(0,1);
    lcd.print("+ or - F Step");
    lcd.setCursor(0,2);
    lcd.print("+ or - R Step");
    lcd.setCursor(0,3);
    lcd.print("10x or 0.1x T Step ");
    
    delay(100);
    int downCount = 0;
    while(digitalRead(RIGHT_SWITCH) == 1) {
      if (map(analogRead(RIGHT_JOY_Y), 0, 1023, 100, -100) == -100) {
        lcd.setCursor(19, downCount);
        lcd.print(" ");
        if (downCount != 3) {
          downCount = downCount + 1;
        } else {
          downCount = 0;
        }
      }
      else if (map(analogRead(RIGHT_JOY_Y), 0, 1023, 100, -100) == 100){
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
      delay(400);
    }
    if (downCount == 0){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("EXITING...");
    }
    if (downCount > 0){
      lcd.clear();
      lcd.setCursor(1,0);
      lcd.print("<- (-)");
      lcd.setCursor(13,0);
      lcd.print("(+) ->");
      delay(300);
      while(true){
        if (digitalRead(RIGHT_SWITCH) == 0) {
          lcd.clear();
          lcd.print("STEP INCREASED");
          break;
        }
        if (digitalRead(LEFT_SWITCH)==0) {
          lcd.clear();
          lcd.print("STEP DECREASED");
          downCount = downCount * -1;
          break;
        }
      }
    }
    joyPosArray[4] = downCount;
    Serial.println(downCount);
    delay(500);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("R/L Flex: ");
    lcd.setCursor(0, 1);
    lcd.print("U/D Flex: ");
    lcd.setCursor(0, 2);
    lcd.print("Rotation: ");
    lcd.setCursor(0, 3);
    lcd.print("Translation: ");
  }

  radio.write(&joyPosArray, sizeof(joyPosArray));  
  radio.startListening();
  delay(20);
  if (radio.available()) {
    int posMetrics[4];
    radio.read(&posMetrics, sizeof(posMetrics));
    topServo = String(posMetrics[0]);
    botServo = String(posMetrics[1]);
    stepper = String(posMetrics[2]);
    linActuator = String(posMetrics[3]);
    for (int i = 0; i < 3; i++){
      if (posMetrics[i]<10){
        lcd.setCursor(11, i);
        lcd.print(" ");
      }
      else if(posMetrics[i]<100){
        lcd.setCursor(12, i);
        lcd.print(" ");
      }
    }
    if (posMetrics[3]<10){
      lcd.setCursor(14, 3);
      lcd.print(" ");
    }
    else if (posMetrics[3]<100){
      lcd.setCursor(15,3);
      lcd.print(" ");
    }
    lcd.setCursor(10, 0);
    lcd.print(topServo + char(223));
    Serial.println("top servo: " + topServo);
    lcd.setCursor(10, 1);
    lcd.print(botServo + char(223));
    Serial.println("bot servo: " + botServo);
    lcd.setCursor(10, 2);
    lcd.print(stepper + char(223));
    Serial.println("stepper: " + stepper);
    lcd.setCursor(13, 3);
    lcd.print(linActuator + " mm");
    Serial.println("lin actuator: " + linActuator);
  }
}
