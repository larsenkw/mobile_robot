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
#include <rc_car/MotorPercent.h>

// Define pins (currently motor A is left, motor B is right)
const int DIRECTION_A = 12;
const int DIRECTION_B = 13;
const int PWM_A = 3;
const int PWM_B = 11;
const int BRAKE_A = 9;
const int BRAKE_B = 8;
const int CURRENT_A = A0;
const int CURRENT_B = A1;

double speed = 0;

// Create ROS Objects
ros::NodeHandle nh;

void motorCallback(const rc_car::MotorPercent& motor_msg)
{
  float left_motor = motor_msg.data[0];
  float right_motor = motor_msg.data[1];

  bool left_direction;
  bool right_direction;
  float left_speed;
  float right_speed;

  if (left_motor >= 0) {
    left_direction = 1;
  }
  else {
    left_direction = 0;
  }

  if (right_motor >= 0) {
    right_direction = 1;
  }
  else {
    right_direction = 0;
  }

  left_speed = fabs(left_motor);
  right_speed = fabs(right_motor);

  setMotorA(left_direction, left_speed, 0);
  setMotorB(right_direction, right_speed, 0);
}

ros::Subscriber<rc_car::MotorPercent> motor_sub("motor_percent", &motorCallback);

void setup() {
  nh.initNode();
  nh.subscribe(motor_sub);
  
  Serial.begin(57600);

  // Setup channel A (left motor)
  pinMode(DIRECTION_A, OUTPUT);
  pinMode(BRAKE_A, OUTPUT);
  
  // Setup channel B (right motor)
  pinMode(DIRECTION_B, OUTPUT);
  pinMode(BRAKE_B, OUTPUT);

}

void loop() {
//  while (Serial.available()) {
//    speed = Serial.parseFloat();
//  }
//
//  setMotorA(true, false, speed);
//  setMotorB(true, false, speed);
//
//  delay(500);

  nh.spinOnce();
  delay(1);
}

// Motor Control Functions
void setMotorA(bool direction, double speed, bool brake) { 
  // Set direction, true/1 is forward, false/0 is backward
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

void setMotorB(bool direction, double speed,  bool brake) {
  // Set direction, true/1 is forward, false/0 is backward
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

