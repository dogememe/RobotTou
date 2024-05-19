#include <L293.h>

const int LEFT_EN = 8;
const int LEFT1 = 9;
const int LEFT2 = 10;
const int RIGHT_EN = 13;
const int RIGHT1 = 11;
const int RIGHT2 = 12;
L293 lmotor(LEFT_EN, LEFT1, LEFT2);
L293 rmotor(RIGHT_EN, RIGHT1, RIGHT2);
int timer = millis();
void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:

}
void runBoth(int val){

  rmotor.forward(val);
  lmotor.forward(val);
  
  
}
void stopBoth(){
    lmotor.stop();
    rmotor.stop();
}
void runtime(float ms, int power){
  runBoth(power);
  delay(ms);
  stopBoth();
}
void Rturn90(){
  timer = millis();
  while ((timer + 530)>millis()){
  lmotor.forward(250);
  rmotor.back(250);
  delay(20);
  stopBoth();
  delay(30);
  }
  stopBoth();
}
void Lturn90(){
  timer = millis();
  while ((timer + 530)>millis()){
  rmotor.forward(250);
  lmotor.back(250);
  delay(20);
  stopBoth();
  delay(30);
  }
  stopBoth();
}

int mils = millis();
void loop() {
  while (millis() < mils+8*1000){
    runBoth(255);
    delay(30);
    stopBoth();
    delay(30);
  }
delay(1000000);
  // put your main code here, to run repeatedly:

}
