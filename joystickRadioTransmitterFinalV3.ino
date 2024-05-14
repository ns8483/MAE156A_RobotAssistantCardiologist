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
#define LEFT_JOY_Y A4
int LEFT_SWITCH = 3;

//joystick limit parameters
int LR_MIN = -100;
int LR_MAX = 100;
int UD_MIN = -100;
int UD_MAX = 100;


void setup() {
  Serial.begin(115200);
  radio.begin();
  radio.openWritingPipe(addresses[1]);
  radio.openReadingPipe(1, addresses[0]);
  radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();

  //setting up LCD display
  lcd.init();
  lcd.clear();         
  lcd.backlight();
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
  //Sends it to radio for communication
  int joyPos[4];
  int joyPosArray[4];
  joyPos[0]= analogRead(RIGHT_JOY_X);
  joyPos[1]= analogRead(RIGHT_JOY_Y);
  joyPos[2] = analogRead(LEFT_JOY_X);
  joyPos[3] = analogRead(LEFT_JOY_Y);
  joyPosArray[0]= map(joyPos[0], 0, 1023, -100, 100);
  joyPosArray[1]= map(joyPos[1], 0, 1023, 100, -100);
  joyPosArray[2]= map(joyPos[2], 0, 1023, 100,-100);
  joyPosArray[3]= map(joyPos[3], 0, 1023, -100,100);
  radio.write(&joyPosArray, sizeof(joyPos));
  radio.startListening();
  while (!radio.available()) {
    Serial.println("RADIO NOT AVAILABLE");
  }
  int posMetrics[4];
  radio.read(&posMetrics, sizeof(posMetrics));
  topServo = String(posMetrics[0]);
  botServo = String(posMetrics[1]);
  stepper = String(posMetrics[2]);
  linActuator = String(posMetrics[3]);
  lcd.setCursor(10, 0);
  lcd.print(topServo);
  lcd.setCursor(10, 1);
  lcd.print(botServo);
  lcd.setCursor(10, 2);
  lcd.print(stepper);
  lcd.setCursor(13, 3);
  lcd.print(linActuator);
  radio.stopListening();
}
