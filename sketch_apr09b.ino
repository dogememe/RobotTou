#include <L293.h>
#include <PID_v1_bc.h>
#include <math.h>

const int leftEncoderPin = 2;
const int rightEncoderPin =  3;

int leftTicks = 0;
int lPrev = 0;
int rightTicks = 0;
int rPrev = 0;


const int counts = 11*20; // encoder counts/rev * gear ratio

const int LEFT_EN = 8; // motor control
const int LEFT1 = 9;
const int LEFT2 = 10;
const int RIGHT_EN = 13;
const int RIGHT1 = 11;
const int RIGHT2 = 12;

const float radius = 30; //define wheel size and spaxing
const float baseRadius = 167/2;

L293 lmotor(LEFT_EN, LEFT1, LEFT2); // Init motors
L293 rmotor(RIGHT_EN, RIGHT1, RIGHT2);

int LInput;
int MSetpoint;
int LOutput;
double xPos = 0;
double yPos = 0;
double Kp = 1;
double Ki = .05;
double Kd = .05;
PID lPID(&LInput, &LOutput, &MSetpoint, Kp, Ki, Kd, DIRECT);
PID rPID(&rInput, &rOutput, &MSetpoint, Kp, Ki, Kd, DIRECT);


void attachInterrupts() { //INIT encoders
  attachInterrupt(digitalPinToInterrupt(leftEncoderPin),leftTicks++, HIGH);
  attachInterrupt(digitalPinToInterrupt(rightEncoderPin),rightTicks++, HIGH);
}


float countsToDist(float C){
  return (C/counts)*2*PI*radius;
}

float calcPos(float pastL, float pastR, float pastTime){
  float vR = countsToDist(rightTicks-pastR);
  float vL = countsToDist(leftTicks-pastL);
  lPrev = leftTicks;
  rPrev = rightTicks;
  float dT = millis()-pastTime;
  float newAngle = ((1/baseRadius)*(vR-vL)*(dT));
  float newX = xPos+(0.5)*baseRadius*((vR+vL)/(vR-vL))*sin(dT*(vR-vL)/baseRadius);
  float newY = yPos+(0.5)*baseRadius*((vR+vL)/(vR-vL))*cos(dT*(vR-vL)/baseRadius);
  
  return newX, newY, newAngle, vR, vL;
}


//PID THINGS

//right motor is master, left is 


void setAngle(float theta){
  
}

void followTrajectory(float X, float Y){
  //float[5] positionData = calcPos();
  float desiredX = xPos;
  float desiredY = yPos;

  
}


void setup() {

}

void loop() {
  lPID.Compute();
  rPID.Compute();
}
