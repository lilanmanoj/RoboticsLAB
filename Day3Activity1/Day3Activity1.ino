/*
 * Day3Activity1.ino
 * Activity 1 - Open-Loop Motor Characterization
 *
 * Applies a sequence of PWM values to ONE wheel at a time, waits for the
 * speed to settle, then measures the steady-state encoder speed for that
 * PWM value. The sweep is run up (0 -> 255) and back down (255 -> 0) so the
 * dead-band and any hysteresis are visible, and is then repeated for the
 * second wheel.
 *
 * Results are streamed over Serial as CSV so they can be captured straight
 * into a file and plotted (RPM vs. PWM). See README.md in this folder.
 *
 * Board: ESP32-S3-DevKitC-1
 * Driver: TB6612FNG   Motors: N20 6V 300RPM w/ hall quadrature encoder
 * Pin mapping from the repository README.md
 */

#include <Arduino.h>

// Declared up here because the Arduino preprocessor injects the auto-generated
// function prototypes ahead of the first function definition - any type used in
// a signature must already be visible at that point.
enum MotorId { MOTOR_LEFT = 0, MOTOR_RIGHT = 1 };
enum Direction { DIR_FORWARD = 0, DIR_REVERSE = 1 };

// ---------------------------------------------------------------------------
// Pin map (repository README.md)
// ---------------------------------------------------------------------------

// NOTE: the motor-to-encoder pairing on this build is crossed relative to the
// table in ../README.md. Verified on the bench with MotorDiag: driving TB6612
// channel A turns the encoder on GPIO 36/37, and channel B turns the encoder on
// GPIO 38/39. The pin numbers below follow the hardware, not the table.

// Motor A - N20 Motor Left (Starboard Side)
#define AIN1 10
#define AIN2 11
#define PWMA 4
#define ENC_L_C1 36  // encoder channel A
#define ENC_L_C2 37  // encoder channel B

// Motor B - N20 Motor Right (Port Side)
#define BIN1 47
#define BIN2 48
#define PWMB 5
#define ENC_R_C1 38  // encoder channel A
#define ENC_R_C2 39  // encoder channel B

// TB6612FNG standby/enable pin (shared by both motors)
#define STBY 1

// ---------------------------------------------------------------------------
// Encoder / gearbox constants  --  CALIBRATE THESE FOR YOUR MOTOR
//
// A typical N20 hall encoder gives 7 pulses per revolution per channel on the
// MOTOR shaft. Decoding both edges of both channels (x4 quadrature) gives
// 28 counts per motor revolution. The gearbox of the 6V/300RPM variant is
// nominally 1:30, so one OUTPUT (wheel) revolution is 28 * 30 = 840 counts.
//
// If your RPM numbers look scaled by a constant factor, this is the constant
// to fix: mark the wheel, turn it by hand exactly 10 turns with the sketch
// in ENCODER_CHECK mode (see below) and divide the printed counts by 10.
// ---------------------------------------------------------------------------
const float ENCODER_PPR_PER_CHANNEL = 7.0f;   // pulses/rev/channel, motor shaft
const float QUADRATURE_MULTIPLIER   = 4.0f;   // x4 decoding (both edges, both ch)
const float GEAR_RATIO              = 30.0f;  // motor revs per output rev

const float COUNTS_PER_MOTOR_REV =
    ENCODER_PPR_PER_CHANNEL * QUADRATURE_MULTIPLIER;             // 28
const float COUNTS_PER_OUTPUT_REV = COUNTS_PER_MOTOR_REV * GEAR_RATIO;  // 840

// Flip to -1 if a motor reports negative RPM while physically driving forward
// (happens when the motor terminals or the encoder channels are swapped).
// Measured with MotorDiag: channel A counts up under forward drive, channel B
// counts down, because both motors' terminals are wired crossed the same way.
const int LEFT_ENCODER_SIGN  = +1;
const int RIGHT_ENCODER_SIGN = -1;

// ---------------------------------------------------------------------------
// Experiment settings
// ---------------------------------------------------------------------------

// PWM values to test (8-bit, 0..255). Fine near the bottom so the dead-band
// is well resolved, coarser at the top where the curve is linear.
const uint8_t PWM_STEPS[] = {
    0, 20, 30, 40, 50, 60, 75, 90, 105, 120, 135, 150, 165, 180, 195, 210,
    225, 240, 255
};
const int NUM_PWM_STEPS = sizeof(PWM_STEPS) / sizeof(PWM_STEPS[0]);

const unsigned long SETTLE_MS      = 1200;  // time to reach steady state
const unsigned long SAMPLE_MS      = 300;   // length of one measurement window
const int           SAMPLES_PER_PT = 3;     // windows averaged per PWM point
const unsigned long REST_MS        = 1500;  // motor off between sweeps/motors

const bool TEST_REVERSE_DIRECTION = false;  // true -> also sweep in reverse
const bool DO_DOWN_SWEEP          = true;   // true -> also sweep 255 -> 0

const uint32_t PWM_FREQ_HZ = 20000;  // 20 kHz: above audible, N20-friendly
const uint8_t  PWM_RES_BITS = 8;     // analogWrite range stays 0..255

const unsigned long SERIAL_BAUD = 115200;

// ---------------------------------------------------------------------------
// Quadrature decoding
// ---------------------------------------------------------------------------

// Index = (previous state << 2) | current state, where state = (A << 1) | B.
// 0 = no change / invalid transition, +/-1 = one count.
static const int8_t QUAD_TABLE[16] = {
    0, -1, +1,  0,
   +1,  0,  0, -1,
   -1,  0,  0, +1,
    0, +1, -1,  0
};

volatile int32_t leftCount  = 0;
volatile int32_t rightCount = 0;
volatile uint8_t leftState  = 0;
volatile uint8_t rightState = 0;

void IRAM_ATTR leftEncoderISR() {
  uint8_t s = (digitalRead(ENC_L_C1) << 1) | digitalRead(ENC_L_C2);
  leftCount += QUAD_TABLE[(leftState << 2) | s];
  leftState = s;
}

void IRAM_ATTR rightEncoderISR() {
  uint8_t s = (digitalRead(ENC_R_C1) << 1) | digitalRead(ENC_R_C2);
  rightCount += QUAD_TABLE[(rightState << 2) | s];
  rightState = s;
}

int32_t readLeftCount() {
  noInterrupts();
  int32_t c = leftCount;
  interrupts();
  return c;
}

int32_t readRightCount() {
  noInterrupts();
  int32_t c = rightCount;
  interrupts();
  return c;
}

// ---------------------------------------------------------------------------
// Motor helpers
// ---------------------------------------------------------------------------

const char *motorName(MotorId m) { return m == MOTOR_LEFT ? "LEFT" : "RIGHT"; }
const char *dirName(Direction d) { return d == DIR_FORWARD ? "FWD" : "REV"; }

void driveMotor(MotorId motor, Direction dir, uint8_t pwm) {
  const int in1 = (motor == MOTOR_LEFT) ? AIN1 : BIN1;
  const int in2 = (motor == MOTOR_LEFT) ? AIN2 : BIN2;
  const int pwmPin = (motor == MOTOR_LEFT) ? PWMA : PWMB;

  // Same convention as TestMotor.ino: IN1 LOW / IN2 HIGH is "forward".
  digitalWrite(in1, dir == DIR_FORWARD ? LOW : HIGH);
  digitalWrite(in2, dir == DIR_FORWARD ? HIGH : LOW);
  analogWrite(pwmPin, pwm);
}

void stopMotor(MotorId motor) {
  const int in1 = (motor == MOTOR_LEFT) ? AIN1 : BIN1;
  const int in2 = (motor == MOTOR_LEFT) ? AIN2 : BIN2;
  const int pwmPin = (motor == MOTOR_LEFT) ? PWMA : PWMB;

  analogWrite(pwmPin, 0);
  digitalWrite(in1, LOW);   // both LOW = coast, avoids a braking transient
  digitalWrite(in2, LOW);
}

void stopBothMotors() {
  stopMotor(MOTOR_LEFT);
  stopMotor(MOTOR_RIGHT);
}

int32_t readCount(MotorId motor) {
  return (motor == MOTOR_LEFT) ? readLeftCount() : readRightCount();
}

int encoderSign(MotorId motor) {
  return (motor == MOTOR_LEFT) ? LEFT_ENCODER_SIGN : RIGHT_ENCODER_SIGN;
}

// ---------------------------------------------------------------------------
// Measurement
// ---------------------------------------------------------------------------

// Measures the steady-state speed at one PWM value and prints one CSV row.
void measurePoint(MotorId motor, Direction dir, const char *sweep, uint8_t pwm) {
  driveMotor(motor, dir, pwm);
  delay(SETTLE_MS);  // let the motor reach steady state before sampling

  float rpmSamples[SAMPLES_PER_PT];
  int32_t totalCounts = 0;
  unsigned long totalMicros = 0;

  for (int i = 0; i < SAMPLES_PER_PT; i++) {
    const int32_t c0 = readCount(motor);
    const unsigned long t0 = micros();
    delay(SAMPLE_MS);
    const int32_t c1 = readCount(motor);
    const unsigned long t1 = micros();

    const int32_t dCounts = (c1 - c0) * encoderSign(motor);
    const unsigned long dMicros = t1 - t0;  // unsigned math wraps correctly

    totalCounts += dCounts;
    totalMicros += dMicros;

    const float countsPerSec = (float)dCounts * 1e6f / (float)dMicros;
    rpmSamples[i] = countsPerSec * 60.0f / COUNTS_PER_OUTPUT_REV;
  }

  float rpmMin = rpmSamples[0];
  float rpmMax = rpmSamples[0];
  float rpmSum = 0.0f;
  for (int i = 0; i < SAMPLES_PER_PT; i++) {
    rpmSum += rpmSamples[i];
    if (rpmSamples[i] < rpmMin) rpmMin = rpmSamples[i];
    if (rpmSamples[i] > rpmMax) rpmMax = rpmSamples[i];
  }

  const float rpmOutput = rpmSum / SAMPLES_PER_PT;   // wheel / gearbox output
  const float rpmMotor  = rpmOutput * GEAR_RATIO;    // motor shaft
  const float rpmSpread = rpmMax - rpmMin;           // steady-state quality
  const float countsPerSec = (float)totalCounts * 1e6f / (float)totalMicros;

  // CSV row - keep the column order in sync with printCsvHeader()
  Serial.print(motorName(motor));      Serial.print(',');
  Serial.print(dirName(dir));          Serial.print(',');
  Serial.print(sweep);                 Serial.print(',');
  Serial.print(pwm);                   Serial.print(',');
  Serial.print(pwm * 100.0f / 255.0f, 1); Serial.print(',');
  Serial.print(totalCounts);           Serial.print(',');
  Serial.print(totalMicros / 1000);    Serial.print(',');
  Serial.print(countsPerSec, 2);       Serial.print(',');
  Serial.print(rpmOutput, 3);          Serial.print(',');
  Serial.print(rpmMotor, 1);           Serial.print(',');
  Serial.println(rpmSpread, 3);
}

void runSweep(MotorId motor, Direction dir, bool ascending) {
  const char *sweep = ascending ? "UP" : "DOWN";

  for (int i = 0; i < NUM_PWM_STEPS; i++) {
    const int idx = ascending ? i : (NUM_PWM_STEPS - 1 - i);
    measurePoint(motor, dir, sweep, PWM_STEPS[idx]);
  }
}

void characterizeMotor(MotorId motor, Direction dir) {
  Serial.print("# --- characterizing motor ");
  Serial.print(motorName(motor));
  Serial.print(" direction ");
  Serial.print(dirName(dir));
  Serial.println(" ---");

  stopBothMotors();
  delay(REST_MS);

  runSweep(motor, dir, true);
  if (DO_DOWN_SWEEP) {
    runSweep(motor, dir, false);
  }

  stopBothMotors();
  delay(REST_MS);
}

// ---------------------------------------------------------------------------
// Reporting
// ---------------------------------------------------------------------------

void printCsvHeader() {
  Serial.println("motor,direction,sweep,pwm,duty_percent,counts,window_ms,"
                 "counts_per_sec,rpm_output,rpm_motor,rpm_spread");
}

void printPreamble() {
  Serial.println();
  Serial.println("# ==========================================================");
  Serial.println("# Day 3 - Activity 1: Open-Loop Motor Characterization");
  Serial.println("# ==========================================================");
  Serial.print("# counts_per_output_rev = ");
  Serial.println(COUNTS_PER_OUTPUT_REV, 1);
  Serial.print("# settle_ms = ");            Serial.println(SETTLE_MS);
  Serial.print("# sample_ms = ");            Serial.println(SAMPLE_MS);
  Serial.print("# samples_per_point = ");    Serial.println(SAMPLES_PER_PT);
  Serial.print("# pwm_freq_hz = ");          Serial.println(PWM_FREQ_HZ);
  Serial.print("# pwm_points = ");           Serial.println(NUM_PWM_STEPS);
  Serial.println("# rpm_output = wheel/gearbox output shaft RPM");
  Serial.println("# rpm_spread = max-min of the per-window RPM samples "
                 "(small => steady state reached)");
  Serial.println("# NOTE: raise the chassis so the wheels spin free.");
  Serial.println("#");
}

void runExperiment() {
  printPreamble();
  printCsvHeader();

  digitalWrite(STBY, HIGH);  // enable TB6612FNG outputs

  characterizeMotor(MOTOR_LEFT, DIR_FORWARD);
  characterizeMotor(MOTOR_RIGHT, DIR_FORWARD);

  if (TEST_REVERSE_DIRECTION) {
    characterizeMotor(MOTOR_LEFT, DIR_REVERSE);
    characterizeMotor(MOTOR_RIGHT, DIR_REVERSE);
  }

  stopBothMotors();
  digitalWrite(STBY, LOW);  // disable driver outputs

  Serial.println("# done. Send 'r' over Serial to run the sweep again.");
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, LOW);
  stopBothMotors();

  pinMode(ENC_L_C1, INPUT_PULLUP);
  pinMode(ENC_L_C2, INPUT_PULLUP);
  pinMode(ENC_R_C1, INPUT_PULLUP);
  pinMode(ENC_R_C2, INPUT_PULLUP);

#if defined(ESP_ARDUINO_VERSION) && \
    ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  // Core 3.x lets analogWrite() keep its 0..255 range at a chosen frequency.
  analogWriteResolution(PWMA, PWM_RES_BITS);
  analogWriteResolution(PWMB, PWM_RES_BITS);
  analogWriteFrequency(PWMA, PWM_FREQ_HZ);
  analogWriteFrequency(PWMB, PWM_FREQ_HZ);
#endif

  // Seed the decoder state so the first edge is not counted as a jump.
  leftState  = (digitalRead(ENC_L_C1) << 1) | digitalRead(ENC_L_C2);
  rightState = (digitalRead(ENC_R_C1) << 1) | digitalRead(ENC_R_C2);

  attachInterrupt(digitalPinToInterrupt(ENC_L_C1), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_C2), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C1), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C2), rightEncoderISR, CHANGE);

  Serial.println();
  Serial.println("# Day3Activity1 ready. Starting in 3 s "
                 "(lift the wheels off the ground).");
  delay(3000);

  runExperiment();
}

void loop() {
  // Re-run on demand so a whole data set can be repeated without re-flashing.
  if (Serial.available()) {
    const int c = Serial.read();
    if (c == 'r' || c == 'R') {
      runExperiment();
    }
  }
}
