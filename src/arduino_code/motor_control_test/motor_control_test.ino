/*
 * Program for testing the function of the motor shield with the two-wheeled chassis
 */


// Define pins
const int DIRECTION_A = 12;
const int DIRECTION_B = 13;
const int PWM_A = 3;
const int PWM_B = 11;
const int BRAKE_A = 9;
const int BRAKE_B = 8;
const int CURRENT_A = A0;
const int CURRENT_B = A1;

void setup() {
  // Setup channel A (left motor)
  pinMode(DIRECTION_A, OUTPUT);
  pinMode(BRAKE_A, OUTPUT);
  
  // Setup channel B (right motor)
  pinMode(DIRECTION_B, OUTPUT);
  pinMode(BRAKE_B, OUTPUT);
}

void loop() {
  // Motor A forward at full speed
  digitalWrite(DIRECTION_A, HIGH);
  digitalWrite(BRAKE_A, LOW);
  analogWrite(PWM_A, 255);
  delay(2000);

  // Engage brake A and wait
  digitalWrite(BRAKE_A, HIGH);
  delay(1000);

  // Motor B forward at full speed
  digitalWrite(DIRECTION_B, HIGH);
  digitalWrite(BRAKE_B, LOW);
  analogWrite(PWM_B, 255);
  delay(2000);

  // Engage brake B and wait
  digitalWrite(BRAKE_B, HIGH);
  delay(1000);

  // turn brake off for A
  digitalWrite(BRAKE_A, LOW);
  delay(2000);

  // Move both motors backwards at half speed
  digitalWrite(DIRECTION_A, LOW);
  digitalWrite(DIRECTION_B, LOW);
  digitalWrite(BRAKE_A, LOW);
  digitalWrite(BRAKE_B, LOW);
  analogWrite(PWM_A, 123);
  analogWrite(PWM_B, 123);
  delay(2000);

  // Stop both and wait
  digitalWrite(BRAKE_A, HIGH);
  digitalWrite(BRAKE_B, HIGH);
  delay(1000);

}
