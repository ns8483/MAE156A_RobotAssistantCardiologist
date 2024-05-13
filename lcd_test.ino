#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x3F for a 16 chars and 2 line display
String topServo;
String botServo;
String stepper;
String linActuator;
int i = 0;

void setup() {
  lcd.init();
  lcd.clear();         
  lcd.backlight();      // Make sure backlight is on
  
  // Print a message on both lines of the LCD.
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
  topServo = String(i);
  botServo = String(i+1);
  stepper = String(i+2);
  linActuator = String(i+3);
  lcd.setCursor(10, 0);
  lcd.print(topServo);
  lcd.setCursor(10, 1);
  lcd.print(botServo);
  lcd.setCursor(10, 2);
  lcd.print(stepper);
  lcd.setCursor(13, 3);
  lcd.print(linActuator);
  i=i+1;
  delay(1000);
}
