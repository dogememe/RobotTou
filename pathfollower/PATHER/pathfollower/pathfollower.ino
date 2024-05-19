#include "L293.h"
#include "PreMo.h"
#include <Arduino.h>

float xTraj[]= {25, 0.25, 0.75, 0.75, 1.25, 1.25, 1.25, 1.75, 1.75, 1.75, 1.25, 0.75, 0.25, 0.25, 0.75};
float yTraj[] =  {25, 0.75, 0.75, 1.25, 1.25, 1.75, 1.25, 0.75, 0.25, 0.75, 1.25, 1.25, 1.25, 1.75, 1.75};
int trajLen = 15;

// MOTOR PINS
const int LEFT_EN = 8;
const int LEFT1 = 9;
const int LEFT2 = 10;
const int RIGHT1 = 11;
const int RIGHT2 = 12;
const int RIGHT_EN = 13;



// ENCODER PINS
const int ENCODER_LEFT_PIN = 2;
const int ENCODER_RIGHT_PIN = 3;



// ROBOT MEASUREMENTS]
const float RADIUS = 30; // wheel radius in mm
const float LENGTH = 167; // wheel base length in mm


// PULSES PER REV
const int PULSES_PER_REV = 600; // number of pulses per revolution of the wheel.


// MAXIMUM MOTOR SPEED VALUE
const int MOTOR_SPEED_MAX = 250;


// SERIAL COMMUNICATION BAUD RATE
const unsigned long BAUD_RATE = 115200;


// PID TUNING
// PID for path following (for turning when followiung path)
const double KP = 1;
const double KD = 1;
// PID for motors (for twist method)
const double KP_motor = 1.5;
const double KI_motor = 0;


// INTERVALS
// MIN_PULSE_INTERVAL is the minimum time in microseconds before registering each encoder pulse.
// This is to compensate for "bouncing" on certain encoders.
const unsigned long MIN_PULSE_INTERVAL = 1;
// When following path, send the location to app at this interval in millisecond.
const unsigned long SEND_INTERVAL = 0;


// ENCODER PULSES
unsigned long leftPulses;
unsigned long rightPulses;


// TIMING VARIABLES
unsigned long prevLeftTime;
unsigned long prevRightTime;
unsigned long prevSendTime;


// PATH FOLLOWING SPEED
// Target speed of path following in percentage.
const int PATH_FOLLOW_SPEED = 50;

// MOTORS
L293 leftMotor(LEFT_EN, LEFT1, LEFT2);
L293 rightMotor(RIGHT_EN, RIGHT1, RIGHT2);

// MOTOR SPEED FUNCTIONS
void setLeftForward(int speed) {
  leftMotor.forward(speed);
}

void setLeftReverse(int speed) {
  leftMotor.back(speed);
}

void setRightForward(int speed) {
  rightMotor.forward(speed);
}

void setRightReverse(int speed) {
  rightMotor.back(speed);
}

void stopMotors() {
  leftMotor.stop();
  rightMotor.stop();
}



// PATH FOLLOWER
MotorManager motorManager(setLeftForward, setLeftReverse, setRightForward, setRightReverse, stopMotors);
EncoderManager encoderManager(&leftPulses, &rightPulses, PULSES_PER_REV);
PreMo premo(RADIUS, LENGTH, KP, KD, KP_motor, KI_motor, &motorManager, &encoderManager);


void pulseLeft() {
      leftPulses++;
      prevRightTime = micros();
}

void pulseRight() {
    rightPulses++;
    prevLeftTime = micros();
}

void attachInterrupts() {
  attachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN), pulseLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN), pulseRight, CHANGE);
}

void detachInterrupts() {
  detachInterrupt(digitalPinToInterrupt(ENCODER_LEFT_PIN));
  detachInterrupt(digitalPinToInterrupt(ENCODER_RIGHT_PIN));
}

void testMotors() {
  Serial.println("MOTORS TEST");

  while (true)
  {
    Serial.println("left motor forward max speed");
    stopMotors();
    setLeftForward(MOTOR_SPEED_MAX);
    delay(5000);

    Serial.println("right motor forward max speed");
    stopMotors();
    setRightForward(MOTOR_SPEED_MAX);
    delay(5000);

    Serial.println("both motors forward max speed");
    setLeftForward(MOTOR_SPEED_MAX);
    setRightForward(MOTOR_SPEED_MAX);
    delay(5000);

    Serial.println("both motors forward 75% speed");
    setLeftForward(MOTOR_SPEED_MAX*0.75);
    setRightForward(MOTOR_SPEED_MAX*0.75);
    delay(5000);
    
    Serial.println("both motors reverse max speed");
    setLeftReverse(MOTOR_SPEED_MAX);
    setRightReverse(MOTOR_SPEED_MAX);
    delay(5000);

    Serial.println("both motors reverse 75% speed");
    setLeftReverse(MOTOR_SPEED_MAX*0.75);
    setRightReverse(MOTOR_SPEED_MAX*0.75);
    delay(5000);

    Serial.println("Observe motors stop");
    stopMotors();
    delay(5000);
  }
}

void testEncoders() {
    Serial.println("ENCODERS TEST");
    Serial.print("left: ");
    Serial.print(leftPulses);
    Serial.print("\tright: ");
    Serial.println(rightPulses);
    delay(50);

}

void setup() {
  pinMode(ENCODER_LEFT_PIN, INPUT);
  pinMode(ENCODER_RIGHT_PIN, INPUT); 
  Serial.begin(BAUD_RATE);
  motorManager.setSpeedLimits(0, MOTOR_SPEED_MAX);
  premo.twistBothMotors(true);
  premo.setPathFollowSpeed(100);

  attachInterrupts();
  premo.setX(0);
  premo.setY(0);
  premo.startPathFollowing(xTraj,yTraj,trajLen);
}

void sendLocation() {
  if (millis() - prevSendTime > SEND_INTERVAL) {
    // Todo sending location should be less code for user.
    float xPos = premo.getX();
    float yPos = premo.getY();
    float heading = premo.getHeading();
    float xGoal = premo.getGoalX();
    float yGoal = premo.getGoalY();
    prevSendTime = millis();
  }
}

void pathFollowing() {
  if (premo.isFollowingPath()) {
    sendLocation();
  }
}

void handleSteering() {
  
}

void loop() {
  premo.forward(10);
  delay(1000);
  premo.continuePathFollowing();
  premo.loop();
}
