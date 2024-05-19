#include <LiquidCrystal_I2C.h>
#include <math.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);


int redLow = 3000;  //range 1
int redHigh = 100000;
int greenLow = 1500;  //range 2
int greenHigh = 3000;
int blueLow = -1000;  //rang3
int blueHigh = 1500;

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
float ka = -0.998958;
float kk = 0.000520185;
float kd = -1074.96;
float ke = 2.71828182846;

float regression(float inny) {
  float reg= 1/(1+ka*pow(ke,kk*inny))+kd;
    return reg;

}

float sample(){
  int readered = 0;
  int logerton = 0;
  for(int i = 0; i<200; i++){
    digitalWrite(blueHigh, HIGH);
    logerton = analogRead(A1);
    if(readered < logerton){
      readered = logerton;
    delay(20);
    }
  }
  return readered;
}


float val = 0;
float ppm = 0;
void loop() {
  lcd.setCursor(0, 0);
  val = (5.0*sample())/1023.0;
  ppm = regression(val);
  light(ppm);
  lcd.print(val);
  lcd.print(" VOLTS");
  lcd.setCursor(0, 1);
  lcd.print(regression(val));
  lcd.print(" PPM");
  for(int i = 0; i<20; i++){
    light(ppm);
    delay(10);
  }
  light(ppm);
  delay(5000);
  lcd.clear();
  // put your main code here, to run repeatedly:
}
