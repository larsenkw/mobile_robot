/*
 * ROS Node for controlling the motor shield on a two-wheeled 
 * robotics platform.
 * 
 * The main node receives input from a Logitech F710 controller and
 * then processes the values and sends the command signal to this 
 * Arduino node to control the wheel motors and other hardware.
 * 
 * ---Topics---
 * /
 */

#include <ros.h>

// Define pins
const int DIRECTION_A = 12;
const int DIRECTION_B = 13;
const int PWM_A = 3;
const int PWM_B = 11;
const int BRAKE_A = 9;
const int BRAKE_B = 8;
const int CURRENT_A = A0;
const int CURRENT_B = A1;

double speed = 0;

void setup() {
  Serial.begin(9600);

  // Setup channel A (left motor)
  pinMode(DIRECTION_A, OUTPUT);
  pinMode(BRAKE_A, OUTPUT);
  
  // Setup channel B (right motor)
  pinMode(DIRECTION_B, OUTPUT);
  pinMode(BRAKE_B, OUTPUT);

}

void loop() {
  while (Serial.available()) {
    speed = Serial.parseFloat();
  }

  setMotorA(true, false, speed);
  setMotorB(true, false, speed);

  delay(500);

}

// Motor Control Functions
void setMotorA(bool direction, bool brake, double speed) { 
  // Set direction
  if (direction == true) {
    digitalWrite(DIRECTION_A, HIGH);
  }
  else {
    digitalWrite(DIRECTION_A, LOW);
  }

  // Set brake
  if (brake == true) {
    digitalWrite(BRAKE_A, HIGH);
  }
  else {
    digitalWrite(BRAKE_A, LOW);
  }

  // Set Speed (speed range: 0-1, pwm range: 0-255)
  // first convert speed to pwm
  int pwm = (speed) * (255.0);
  // then set speed
  analogWrite(PWM_A, pwm);

  // Print out values
  Serial.println("PWM A: " + String(pwm));
  Serial.println("Brake: " + String(brake));
  Serial.println("Dir: " + String(direction));
}

void setMotorB(bool direction, bool brake, double speed) {
  // Set direction
  if (direction == true) {
    digitalWrite(DIRECTION_B, HIGH);
  }
  else {
    digitalWrite(DIRECTION_B, LOW);
  }

  // Set brake
  if (brake == true) {
    digitalWrite(BRAKE_B, HIGH);
    Serial.println("Brake engaged");
  }
  else {
    digitalWrite(BRAKE_B, LOW);
    Serial.println("No brake");
  }

  // Set Speed (speed range: 0-1, pwm range: 0-255)
  // first convert speed to pwm
  int pwm = (speed) * (255.0);
  // then set speed
  analogWrite(PWM_B, pwm);
  Serial.println("PWM B: " + String(pwm));
}

