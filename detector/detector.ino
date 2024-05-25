#include <LiquidCrystal_I2C.h>
#include <math.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);


int redLow = 3000;  //range 1
int redHigh = 10000;
int greenLow = 1500;  //range 2
int greenHigh = 3000;
int blueLow = 0;  //rang3
int blueHigh = 1500;

int redPin = 3;
int greenPin = 4;
int bluePin = 5;

void setup() {
  lcd.init();
  lcd.backlight();
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  Serial.begin(9600);
}
void light(int inter) {
  if (inter > redLow) {
    Serial.print((inter <= redHigh));
    
    if(inter <= redHigh){
      Serial.print("r2");
    digitalWrite(redPin, HIGH);
    Serial.print("RED");
    }
  }
  if ((greenLow > greenLow)) {
    if (inter <= greenHigh){
      digitalWrite(greenPin, HIGH);
    }
  }
  if (inter > blueLow) {
    if(inter <= blueHigh) {
    digitalWrite(bluePin, HIGH);
    }
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
  val = 1; //(5.0*sample())/1023.0;
  ppm = 4010;//regression(val);
  lcd.print(val);
  //lcd.print(" VOLTS");
  lcd.setCursor(0, 1);
  //lcd.print(regression(val));
  lcd.print("PPM");
  light(3001);
  delay(5000);
  // put your main code here, to run repeatedly:
}
