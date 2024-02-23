#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,  16, 2);

void setup() {
  lcd.init();
  lcd.backlight();
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  Serial.begin(9600);
  delay(1000);
  lcd.print("TEST MODE");
  delay(1000);
  for (int i = 0; i<3; i++){
    digitalWrite(2,HIGH);
    delay(25);
    digitalWrite(2,LOW);
    digitalWrite(3,HIGH);
    delay(25);
    digitalWrite(3,LOW);
    digitalWrite(4,HIGH);
    delay(25);
    digitalWrite(4,LOW);
    digitalWrite(5,HIGH);
    delay(25);
    digitalWrite(5,LOW);
    lcd.noBacklight();
    delay(50);
    
    
    
    
    lcd.backlight();
    delay(50);
  }
  lcd.clear();

    digitalWrite(2,HIGH);
    digitalWrite(3,HIGH);
    digitalWrite(4,HIGH);
    digitalWrite(5,HIGH);
    delay(500);
    digitalWrite(5,LOW);
    digitalWrite(3,LOW);
    digitalWrite(4,LOW);
    digitalWrite(2,LOW);
    delay(1000);
    digitalWrite(2,HIGH);
    digitalWrite(3,HIGH);
    digitalWrite(4,HIGH);
    digitalWrite(5,HIGH);
    delay(500);
    digitalWrite(5,LOW);
    digitalWrite(3,LOW);
    digitalWrite(4,LOW);
    digitalWrite(2,LOW);
}

void loop() {
  lcd.setCursor(0,0);
  lcd.clear();
  lcd.print(5.0*analogRead(0)/1023.0);
  delay(100);
  // put your main code here, to run repeatedly:

}
