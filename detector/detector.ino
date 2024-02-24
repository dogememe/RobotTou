#include <LiquidCrystal_I2C.h>
#include <math.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);


int redLow = 0;  //range 1
int redHigh = 1000;
int greenLow = 1000;  //range 2
int greenHigh = 3000;
int blueLow = 3000;  //rang3
int blueHigh = 5000;

int redPin = 2;
int greenPin = 3;
int bluePin = 4;


void setup() {
  lcd.init();
  lcd.backlight();
  pinMode(redPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  Serial.begin(9600);
}
void light(int inter) {
  if ((inter > redLow) && (inter <= redHigh)) {
    digitalWrite(redHigh, HIGH);
  } else {
    digitalWrite(redHigh, LOW);
  }
  if ((greenLow > greenLow) && (inter <= greenHigh)) {
    digitalWrite(greenHigh, HIGH);
  } else {
    digitalWrite(greenHigh, LOW);
  }
  if ((inter > blueLow) && (inter <= blueHigh)) {
    digitalWrite(blueHigh, HIGH);
  } else {
    digitalWrite(blueHigh, LOW);
  }
}
float regression(int inny) {
  int reg= 0.00129345*pow(inny,3) + 0.608125*pow(inny,2) +14.3137*inny -28.714;

  if (reg <=150){
    return 0;
  }
  else if (reg >=4600){
    return 5000;
  }
  else{
    return reg;
  }

}

int val = 0;
void loop() {
  lcd.clear();
  lcd.setCursor(0, 0);
  val = analogRead(A1);
  lcd.print(5.0 * val / 1023.0);
  lcd.print(" VOLTS");
  lcd.setCursor(0, 1);
  lcd.print(regression(val));
  lcd.print(" PPM");
  light(regression(val));
  delay(3000);
  // put your main code here, to run repeatedly:
}
