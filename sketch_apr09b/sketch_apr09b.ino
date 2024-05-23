#include <L293.h>
#include <PID_v1.h>
#include <math.h>
/*
  left turns are positive direction, right turns are negative






*/

#define PI 3.14159
const int leftEncoderPin = 2;
const int rightEncoderPin = 3;

int leftTicks = 0;
int lPrev = 0;
int rightTicks = 0;
int rPrev = 0;


const int counts = 1248;  // encoder counts/rev * gear ratio

const int LEFT_EN = 13;  // motor control
const int LEFT1 = 11;
const int LEFT2 = 12;
const int RIGHT_EN = 8;
const int RIGHT1 = 10;
const int RIGHT2 = 9;

const float radius = 68/2;  //define wheel size and spaxing
const float baseRadius = (147+14) / 2;

L293 lmotor(LEFT_EN, LEFT1, LEFT2);  // Init motors
L293 rmotor(RIGHT_EN, RIGHT1, RIGHT2);

double Input;
double Setpoint;
double Output;
double xPos = 0;
double yPos = 0;
double Kp = 1;
double Ki = 10;
double Kd = .2;


void incL() {
  leftTicks += 1*lmotor.isForward();
}
void incR() {
  rightTicks += 1;
}
void attachInterrupts() {  //INIT encoders
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin), incL, HIGH);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin), incR, HIGH);
}


float countsToDist(float C) {
  return (C / counts) * 2 * PI * radius;
}

float calcPos(float pastL, float pastR, float pastTime) {
  float vR = countsToDist(rightTicks - pastR);
  float vL = countsToDist(leftTicks - pastL);
  lPrev = leftTicks;
  rPrev = rightTicks;
  float dT = millis() - pastTime;
  float newAngle = ((1 / baseRadius) * (vR - vL) * (dT));
  float newX = xPos + (0.5) * baseRadius * ((vR + vL) / (vR - vL)) * sin(dT * (vR - vL) / baseRadius);
  float newY = yPos + (0.5) * baseRadius * ((vR + vL) / (vR - vL)) * cos(dT * (vR - vL) / baseRadius);

  return newX, newY, newAngle, vR, vL;
}


//PID THINGS
PID testPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
//right motor is master, left is

void moveMotor(double distance){ // move in a straight line in mm 
  
  Setpoint = double(int(leftTicks + counts*distance/(PI*2*radius)));
  Serial.println(Setpoint);
  motor(255);
  while (leftTicks < Setpoint){
    Input = leftTicks;
    testPID.Compute();
    if (Output < 150 && Output >-150){
      stop();
    }
    else{
      motor(Output);
    }
    delay(1);
    Serial.println(Output);
  }
  stop();
  Serial.print("done");
}

void setup() {
  Serial.begin(9600);
  attachInterrupts();
  testPID.SetMode(AUTOMATIC);
  testPID.SetOutputLimits(-255, 255);
}

void turn(double angle){
  Setpoint = double(int(leftTicks + (2*2*PI*baseRadius*angle/360)*counts/(PI*2*radius)));
  Serial.println(Setpoint);
  lmotor.forward(255);
  while (leftTicks < Setpoint){
    Input = leftTicks;
    testPID.Compute();
    if (Output < 150 && Output >-150){
      lmotor.stop();
    }
    else{
      lmotor.forward(Output);
    }
    delay(1);
    Serial.println(Output);
  }
}

void motor(double pow){
  lmotor.forward(pow);
  rmotor.forward(pow);
}
void stop(){
  lmotor.stop();
  rmotor.stop();
}

void loop() {
  turn(90);
  delay(5000);
  //lPID.Compute();
  //rPID.Compute();

 }
