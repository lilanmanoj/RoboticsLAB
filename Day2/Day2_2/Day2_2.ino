#include <VL53L0X.h>



// ============================================================
// EA3121 Robotics Lab 02 - Activity 2 (FIXED SENSOR VERSION)
// Obstacle Avoidance, Tank-Turn Mapping, and Wall Following
//
// ESP32 + VL53L0X ToF Sensor + TB6612FNG (No Servo)
// ============================================================

#include <Wire.h>



// ============================================================
// ACTIVITY SELECTOR
// Change this number to run the different parts of Activity 2!
// 1 = Part I: Obstacle Avoidance (Sensor facing FORWARD)
// 2 = Part II: 2D Mapping (Robot spins in place to map)
// 3 = Part III: Wall Following (Sensor must face SIDEWAYS to the wall)
// ============================================================
const int ROBOT_MODE = 1; 

// ============================================================
// TB6612FNG CONNECTIONS
// ============================================================
#define STBY_PIN 21

// Left Motor - Motor A
#define PWMA_PIN 22
#define AIN1_PIN 16
#define AIN2_PIN 17

// Right Motor - Motor B
#define PWMB_PIN 23
#define BIN1_PIN 18
#define BIN2_PIN 19

// ============================================================
// SENSOR CONNECTIONS (VL53L0X)
// ============================================================
// Using alternative I2C pins because 21/22 are used by motors
#define I2C_SDA 32
#define I2C_SCL 33

VL53L0X sensor;

// ============================================================
// SETTINGS & VARIABLES
// ============================================================
int BASE_SPEED = 210;
int TURN_SPEED = 180;

// Wall Following PD Controller Variables
float Kp_Wall = 1.5;
float Kd_Wall = 2.0;
float previousError = 0;
int TARGET_DISTANCE_MM = 400; // 40 cm from the wall


// ============================================================
// MOTOR CONTROL ENGINE
// ============================================================
void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
    analogWrite(PWMA_PIN, speed);
  } else if (speed < 0) {
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
    analogWrite(PWMA_PIN, -speed);
  } else {
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, HIGH);
    analogWrite(PWMA_PIN, 255);
  }
}

void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
    analogWrite(PWMB_PIN, speed);
  } else if (speed < 0) {
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
    analogWrite(PWMB_PIN, -speed);
  } else {
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, HIGH);
    analogWrite(PWMB_PIN, 255);
  }
}

void driveMotors(int leftSpeed, int rightSpeed) {
  setLeftMotor(leftSpeed);
  setRightMotor(rightSpeed);
}

void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
}


// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);

  // Initialize Motor Pins
  pinMode(STBY_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH); // Wake up driver
  stopMotors();

  // Initialize VL53L0X on Custom I2C Pins
  Wire.begin(I2C_SDA, I2C_SCL);
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize VL53L0X!");
    while (1) {}
  }
  
  // Start continuous back-to-back mode
  sensor.startContinuous();

  Serial.println("======================================");
  Serial.print("FIXED SENSOR ACTIVITY 2 - MODE: ");
  Serial.println(ROBOT_MODE);
  Serial.println("======================================");
  delay(2000); // Wait 2 seconds before starting
}


// ============================================================
// MAIN CONTROL LOOP
// ============================================================
void loop() {
  if (ROBOT_MODE == 1) {
    runObstacleAvoidance();
  } 
  else if (ROBOT_MODE == 2) {
    run2DMapping();
  } 
  else if (ROBOT_MODE == 3) {
    runWallFollower();
  }
}


// ============================================================
// PART I: OBSTACLE AVOIDANCE
// (Sensor must physically point forward)
// ============================================================
void runObstacleAvoidance() {
  int distance = sensor.readRangeContinuousMillimeters();

  if (distance < 250 && !sensor.timeoutOccurred()) { 
    // Obstacle is closer than 25 cm
    stopMotors();
    Serial.println("Obstacle Detected! Reversing and Turning...");
    
    // Reverse slightly
    driveMotors(-BASE_SPEED, -BASE_SPEED);
    delay(400);
    
    // Spin right to avoid
    driveMotors(TURN_SPEED, -TURN_SPEED);
    delay(600);
  } else {
    // Path is clear
    driveMotors(BASE_SPEED, BASE_SPEED);
  }
  delay(30);
}


// ============================================================
// PART II: 2D MAPPING (NO SERVO)
// Robot spins 360 degrees using motors to scan the room
// ============================================================
void run2DMapping() {
  Serial.println("--- Starting 360 Spin Scan ---");
  
  unsigned long scanStartTime = millis();
  
  // Start spinning the robot in place
  driveMotors(TURN_SPEED, -TURN_SPEED); 
  
  // Spin and scan for roughly 3 seconds (adjust time based on motor speed)
  while(millis() - scanStartTime < 3000) {
    int distance = sensor.readRangeContinuousMillimeters();
    
    // Print the time offset and distance
    Serial.print(millis() - scanStartTime);
    Serial.print(" ms, Distance: ");
    Serial.print(distance);
    Serial.println(" mm");
    
    delay(50); // Small delay between rapid readings
  }
  
  stopMotors();
  Serial.println("--- Spin Scan Complete ---");
  
  delay(10000); // Wait 10 seconds before the next scan
}


// ============================================================
// PART III: WALL FOLLOWING (Right Wall)
// (Sensor must physically point SIDEWAYS towards the wall)
// Maintains 40cm (400mm) distance
// ============================================================
void runWallFollower() {
  int currentDistance = sensor.readRangeContinuousMillimeters();
  
  // Ignore readings that are out of bounds or errors
  if (currentDistance > 1200 || sensor.timeoutOccurred()) {
    // Lost the wall - drive in a slight rightward curve to find it
    driveMotors(BASE_SPEED, BASE_SPEED - 50); 
    return;
  }

  // Calculate Error (Target is 400mm)
  float error = TARGET_DISTANCE_MM - currentDistance;
  
  // PD Calculation
  float derivative = error - previousError;
  float correction = (Kp_Wall * error) + (Kd_Wall * derivative);
  previousError = error;

  // Limit correction to prevent erratic spinning
  correction = constrain(correction, -100, 100);

  // Apply to motors
  // If error is positive (too close to wall), correction is positive
  // Right wheel speeds up, left wheel slows down to turn away
  int leftSpeed = BASE_SPEED - correction;
  int rightSpeed = BASE_SPEED + correction;

  driveMotors(leftSpeed, rightSpeed);
  delay(20);
}