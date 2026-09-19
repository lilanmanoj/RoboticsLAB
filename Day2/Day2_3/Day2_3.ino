#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ============================================================
// EA3121 ROBOTICS
// SMOOTH + FAST PID WALL FOLLOWING ROBOT
//
// VL53L0X faces RIGHT side of robot
// Target wall distance = 400 mm = 40 cm
//
// ESP32 + VL53L0X + TB6612FNG
// ============================================================


// ============================================================
// VL53L0X
// ============================================================

#define SDA_PIN 32
#define SCL_PIN 33

Adafruit_VL53L0X lox;


// ============================================================
// TB6612FNG MOTOR DRIVER
// ============================================================

#define STBY_PIN 21

// Left Motor
#define AIN1_PIN 16
#define AIN2_PIN 17
#define PWMA_PIN 22

// Right Motor
#define BIN1_PIN 18
#define BIN2_PIN 19
#define PWMB_PIN 23


// ============================================================
// SPEED SETTINGS
// ============================================================

// Normal forward speed
int BASE_SPEED = 220;

// Maximum PID steering correction
int MAX_CORRECTION = 100;

// Minimum allowed forward PWM
int MIN_MOTOR_SPEED = 80;

// Maximum allowed normal PWM
int MAX_MOTOR_SPEED = 200;
#define MAX_PWM        255


// ============================================================
// WALL FOLLOWING TARGET
// ============================================================

// 400 mm = 40 cm
const float TARGET_DISTANCE = 400.0;

// Ignore tiny measurement errors around target
const float DEAD_BAND = 8.0;


// ============================================================
// SAFETY DISTANCES
// ============================================================

// Start stronger wall avoidance below 30 cm
const float CLOSE_DISTANCE = 300.0;

// Emergency correction below 18 cm
const float EMERGENCY_DISTANCE = 180.0;


// ============================================================
// PID PARAMETERS
// ============================================================

float Kp = 0.32;
float Ki = 0.000;
float Kd = 0.010;


// ============================================================
// PID VARIABLES
// ============================================================

float error = 0.0;
float previousError = 0.0;
float integral = 0.0;
float rawDerivative = 0.0;
float filteredDerivative = 0.0;
float correction = 0.0;
float smoothCorrection = 0.0;
unsigned long previousPIDTime = 0;


// ============================================================
// INTEGRAL LIMIT
// ============================================================

const float MAX_INTEGRAL = 500.0;


// ============================================================
// DISTANCE FILTER
// ============================================================

float filteredDistance = 0.0;
bool filterInitialized = false;
const float FILTER_ALPHA = 0.72;


// ============================================================
// DERIVATIVE FILTER
// ============================================================

const float DERIVATIVE_ALPHA = 0.75;


// ============================================================
// STEERING CORRECTION FILTER
// ============================================================

const float CORRECTION_ALPHA = 0.55;


// ============================================================
// INVALID READING CONTROL
// ============================================================

int invalidReadingCount = 0;
const int MAX_INVALID_BEFORE_SEARCH = 3;


// ============================================================
// LEFT MOTOR
// ============================================================

void setLeftMotor(int speed) {

  speed = constrain(speed, -MAX_PWM, MAX_PWM);

  if (speed > 0) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
    analogWrite(PWMA_PIN, speed);
  }
  else if (speed < 0) {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
    analogWrite(PWMA_PIN, -speed);
  }
  else {
    // Short brake
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, HIGH);
    analogWrite(PWMA_PIN, 255);
  }
}


// ============================================================
// RIGHT MOTOR
// ============================================================

void setRightMotor(int speed) {

  speed = constrain(speed, -MAX_PWM, MAX_PWM);

  if (speed > 0) {
    // Forward
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
    analogWrite(PWMB_PIN, speed);
  }
  else if (speed < 0) {
    // Reverse
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
    analogWrite(PWMB_PIN, -speed);
  }
  else {
    // Short brake
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, HIGH);
    analogWrite(PWMB_PIN, 255);
  }
}


// ============================================================
// DRIVE MOTORS
// ============================================================

void driveMotors(int leftSpeed, int rightSpeed) {
  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}


// ============================================================
// STOP MOTORS
// ============================================================

void stopMotors() {
  driveMotors(0, 0);
}


// ============================================================
// READ VL53L0X
// ============================================================

int readDistance() {
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false);

  if (measure.RangeStatus != 4) {
    int distance = measure.RangeMilliMeter;

    if (distance >= 30 && distance <= 2000) {
      return distance;
    }
  }

  return -1;
}


// ============================================================
// RESET PID
// ============================================================

void resetPID() {
  error = 0.0;
  previousError = 0.0;
  integral = 0.0;
  rawDerivative = 0.0;
  filteredDerivative = 0.0;
  correction = 0.0;
  smoothCorrection = 0.0;
  filterInitialized = false;
  invalidReadingCount = 0;
  previousPIDTime = micros();
}


// ============================================================
// WALL FOLLOWING PID
// ============================================================

void wallFollowPID(float distance) {

  // ==========================================================
  // FIRST SENSOR READING
  // ==========================================================

  if (!filterInitialized) {
    filteredDistance = distance;
    filterInitialized = true;
    previousError = filteredDistance - TARGET_DISTANCE;
    previousPIDTime = micros();

    // Start moving straight
    driveMotors(BASE_SPEED, BASE_SPEED);
    return;
  }


  // ==========================================================
  // FILTER DISTANCE
  // ==========================================================

  filteredDistance = (FILTER_ALPHA * filteredDistance) + ((1.0 - FILTER_ALPHA) * distance);

  // ==========================================================
  // TIME CALCULATION
  // ==========================================================

  unsigned long now = micros();
  float dt = (now - previousPIDTime) / 1000000.0;
  previousPIDTime = now;

  if (dt < 0.005 || dt > 0.10) {
    dt = 0.020;
  }

  // ==========================================================
  // ERROR
  // ==========================================================
  error = filteredDistance - TARGET_DISTANCE;

  // ==========================================================
  // DEAD BAND
  // ==========================================================
  if (fabs(error) < DEAD_BAND) {
    error = 0.0;
  }

  // ==========================================================
  // INTEGRAL
  // ==========================================================
  if (fabs(error) < 120.0) {
    integral += error * dt;
  } else {
    integral *= 0.90;
  }

  integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);

  // ==========================================================
  // DERIVATIVE
  // ==========================================================
  rawDerivative = (error - previousError) / dt;
  filteredDerivative = (DERIVATIVE_ALPHA * filteredDerivative) + ((1.0 - DERIVATIVE_ALPHA) * rawDerivative);
  previousError = error;

  // ==========================================================
  // PID OUTPUT
  // ==========================================================
  correction = (Kp * error) + (Ki * integral) + (Kd * filteredDerivative);
  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  // ==========================================================
  // SMOOTH PID OUTPUT
  // ==========================================================
  smoothCorrection = (CORRECTION_ALPHA * smoothCorrection) + ((1.0 - CORRECTION_ALPHA) * correction);

  // ==========================================================
  // CLOSE TO WALL
  // ==========================================================
  if (filteredDistance < CLOSE_DISTANCE) {
    float closeCorrection = map((int)filteredDistance, 150, 300, -70, -35);
    closeCorrection = constrain(closeCorrection, -70, -35);

    if (closeCorrection < smoothCorrection) {
      smoothCorrection = (0.45 * smoothCorrection) + (0.55 * closeCorrection);
    }
  }

  // ==========================================================
  // EMERGENCY WALL AVOIDANCE
  // ==========================================================
  if (filteredDistance < EMERGENCY_DISTANCE) {
    driveMotors(60, 170);
    integral = 0.0;

    static unsigned long emergencyTimer = 0;
    if (millis() - emergencyTimer > 200) {
      emergencyTimer = millis();
      Serial.println("EMERGENCY -> STRONG LEFT");
    }
    return;
  }

  // ==========================================================
  // MOTOR MIXING
  // ==========================================================
  int leftSpeed = BASE_SPEED + smoothCorrection;
  int rightSpeed = BASE_SPEED - smoothCorrection;

  // ==========================================================
  // MOTOR LIMITS
  // ==========================================================
  leftSpeed = constrain(leftSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

  // ==========================================================
  // DRIVE ROBOT
  // ==========================================================
  driveMotors(leftSpeed, rightSpeed);

  // ==========================================================
  // SERIAL DEBUG
  // ==========================================================
  static unsigned long debugTimer = 0;
  if (millis() - debugTimer >= 100) {
    debugTimer = millis();

    Serial.print("Raw:"); Serial.print(distance, 0);
    Serial.print(" | Filter:"); Serial.print(filteredDistance, 1);
    Serial.print(" | Error:"); Serial.print(error, 1);
    Serial.print(" | D:"); Serial.print(filteredDerivative, 1);
    Serial.print(" | PID:"); Serial.print(correction, 1);
    Serial.print(" | Smooth:"); Serial.print(smoothCorrection, 1);
    Serial.print(" | L:"); Serial.print(leftSpeed);
    Serial.print(" | R:"); Serial.println(rightSpeed);
  }
}


// ============================================================
// WALL LOST
// ============================================================

void searchForWall() {
  // Gentle turn RIGHT because wall should be on right side.
  driveMotors(115, 90);
}


// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  // ==========================================================
  // MOTOR DRIVER PINS
  // ==========================================================
  pinMode(STBY_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  
  // ADDED: Initialize PWM pins as outputs
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);

  // Enable TB6612FNG
  digitalWrite(STBY_PIN, HIGH);
  stopMotors();

  // ==========================================================
  // I2C
  // ==========================================================
  Wire.begin(SDA_PIN, SCL_PIN);

  // ==========================================================
  // VL53L0X
  // ==========================================================
  if (!lox.begin()) {
    Serial.println("ERROR: VL53L0X NOT DETECTED");
    while (1) {
      stopMotors();
      delay(100);
    }
  }

  // ==========================================================
  // READY
  // ==========================================================
  Serial.println("\n======================================");
  Serial.println("FAST + SMOOTH PID WALL FOLLOWER");
  Serial.println("RIGHT SIDE WALL");
  Serial.println("TARGET = 400 mm");
  Serial.println("======================================");
  Serial.print("Kp = "); Serial.println(Kp);
  Serial.print("Ki = "); Serial.println(Ki);
  Serial.print("Kd = "); Serial.println(Kd);

  resetPID();
  delay(1000);
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  int distance = readDistance();

  // ==========================================================
  // VALID READING
  // ==========================================================
  if (distance > 0) {
    invalidReadingCount = 0;
    wallFollowPID(distance);
  }

  // ==========================================================
  // INVALID READING
  // ==========================================================
  else {
    invalidReadingCount++;

    if (invalidReadingCount > MAX_INVALID_BEFORE_SEARCH) {
      integral = 0.0;
      filteredDerivative = 0.0;
      searchForWall();

      static unsigned long lostTimer = 0;
      if (millis() - lostTimer >= 300) {
        lostTimer = millis();
        Serial.println("Wall lost -> gentle RIGHT search");
      }
    }
  }

  // Approximately 50 Hz control
  delay(20);
}