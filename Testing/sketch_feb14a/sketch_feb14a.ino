#include <L293.h>

const int LEFT_EN = 8;
const int LEFT1 = 9;
const int LEFT2 = 10;
const int RIGHT_EN = 13;
const int RIGHT1 = 11;
const int RIGHT2 = 12;
L293 lmotor(LEFT_EN, LEFT1, LEFT2);
L293 rmotor(RIGHT_EN, RIGHT1, RIGHT2);

int lspin = 0;
int rspin = 0;
int lprev = 0;
int rprev = 0;

float kp = 1;
float kd = 10;

float radius = 30;
float width = 150;

int lspeed = 0;
int rspeed = 0;
int counts = 1200;


void Rturn(float angle){
  lprev = lspin;
  float dist = 2*3.14159*width*angle/360;
  float countsToTurn = counts*dist/(2*3.14159*radius);
  while (lspin < (lprev + countsToTurn)){
    Serial.print(lspin-lprev-countsToTurn);
    lmotor.forward(255);
    delay(10);
    lmotor.stop();
    delay(10);
  }
  lmotor.stop();
}

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(2), Ltrack, HIGH);
  attachInterrupt(digitalPinToInterrupt(3), Rtrack, HIGH);
  // put your setup code here, to run once:

}

void Ltrack(){
  lspin++;
}
void Rtrack(){
  rspin++;
}

void run(int speed){
  lmotor.forward(speed);
  rmotor.forward(speed);
}
int lm = 0;
int lm2 = 0;
void loop() {
  Rturn(90);
  Serial.print("done");
  delay(10000);
  // put your main code here, to run repeatedly:

}
