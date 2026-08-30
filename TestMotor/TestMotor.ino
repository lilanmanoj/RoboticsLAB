/*
 * TestMotor.cpp
 * Ramps both N20 gear motors (Left = Motor A, Right = Motor B on the
 * TB6612FNG driver) up from PWM 100 to 255 over 5 seconds, then ramps
 * down from 255 to 0 over 8 seconds, then stops. Runs once in setup();
 * loop() is intentionally empty.
 *
 * Board: ESP32-S3-DevKitC-1
 * Pin mapping in README.md
 */

#include <Arduino.h>

// Motor A - N20 Motor Left (Starboard Side)
#define AIN1 10
#define AIN2 11
#define PWMA 4

// Motor B - N20 Motor Right (Port Side)
#define BIN1 47
#define BIN2 48
#define PWMB 5

// TB6612FNG standby/enable pin (shared by both motors)
#define STBY 1

const int MIN_SPEED = 100;
const int MAX_SPEED = 255;
const unsigned long RAMP_UP_MS = 5000;
const unsigned long RAMP_DOWN_MS = 8000;

void setMotorSpeed(int pwmPin, int in1Pin, int in2Pin, int speed) {
  // speed > 0 drives the motor forward; direction pins fixed for this test
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  analogWrite(pwmPin, speed);
}

void setup() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH); // enable TB6612FNG outputs

  // Ramp up: PWM 100 -> 255 over 5 seconds
  int upSteps = MAX_SPEED - MIN_SPEED;
  unsigned long upStepDelay = RAMP_UP_MS / upSteps;
  for (int speed = MIN_SPEED; speed <= MAX_SPEED; speed++) {
    setMotorSpeed(PWMA, AIN1, AIN2, speed);
    setMotorSpeed(PWMB, BIN1, BIN2, speed);
    delay(upStepDelay);
  }

  // Ramp down: PWM 255 -> 0 over 8 seconds
  int downSteps = MAX_SPEED;
  unsigned long downStepDelay = RAMP_DOWN_MS / downSteps;
  for (int speed = MAX_SPEED; speed >= 0; speed--) {
    setMotorSpeed(PWMA, AIN1, AIN2, speed);
    setMotorSpeed(PWMB, BIN1, BIN2, speed);
    delay(downStepDelay);
  }

  // Full stop
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(STBY, LOW); // disable driver outputs
}

void loop() {
  // Test runs once in setup(); nothing to do here.
}
