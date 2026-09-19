/*
 * RobotBase.h
 * Shared drivetrain core for the EA 3121 Lab 3 sketches: pin map, calibration
 * constants, x4 quadrature encoder decoding, motor output, and wheel velocity
 * measurement.
 *
 * An identical copy of this file lives in every sketch folder so each project
 * stays independently compilable with arduino-cli. If you change a calibration
 * constant, change it in all of them.
 *
 * Board: ESP32-S3-DevKitC-1   Driver: TB6612FNG   Motors: N20 + hall encoder
 */

#ifndef ROBOT_BASE_H
#define ROBOT_BASE_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Pin map
//
// NOTE: the motor-to-encoder pairing is crossed relative to the repository
// README.md. Verified on the bench: TB6612FNG channel A turns the encoder on
// GPIO 36/37, channel B turns the encoder on GPIO 38/39. These follow the
// hardware, not the table.
// ---------------------------------------------------------------------------

// Motor A - N20 Motor Left (Starboard Side)
#define AIN1 10
#define AIN2 11
#define PWMA 4
#define ENC_L_C1 36
#define ENC_L_C2 37

// Motor B - N20 Motor Right (Port Side)
#define BIN1 47
#define BIN2 48
#define PWMB 5
#define ENC_R_C1 38
#define ENC_R_C2 39

#define STBY 1

// ---------------------------------------------------------------------------
// Calibration  --  VERIFY THESE THREE BEFORE TRUSTING ANY NUMBER
// ---------------------------------------------------------------------------

// 7 PPR/channel x4 quadrature x 30:1 gearbox. Activity 1 measured ~420 RPM at
// full duty on a 5 V rail, which is high for a 300 RPM/6 V motor, so this is
// probably wrong. Calibrate: mark a wheel, turn it exactly 10 revolutions by
// hand, counts/10 is the true value. If the encoder is 11 PPR it is 1320.
static const float COUNTS_PER_OUTPUT_REV = 840.0f;

// ASSUMPTION: 43 mm wheels. Not recorded anywhere in the repository - measure
// yours. Every m/s figure scales with it; RPM figures do not.
static const float WHEEL_DIAMETER_M = 0.043f;
static const float WHEEL_RADIUS_M = WHEEL_DIAMETER_M / 2.0f;

// ASSUMPTION: 100 mm between wheel contact patches. Measure centre-to-centre
// across your chassis. Only affects the differential-drive kinematics.
static const float WHEEL_BASE_M = 0.100f;

// Encoder polarity: both motors' terminals are wired crossed the same way, so
// forward drive counts up on channel A and down on channel B.
static const int ENCODER_SIGN[2] = {+1, -1};

// Feedforward from Activity 1 (day3_activity1.csv, 2026-08-30):
//   pwm = FF_SLOPE * rpm + FF_INTERCEPT
// The fitted dead-band understates reality at low speed - the left wheel does
// not break away until PWM ~50 - so expect poor open-loop tracking below about
// 90 RPM on the left.
static const float FF_SLOPE[2] = {0.585f, 0.605f};
static const float FF_INTERCEPT[2] = {9.6f, 10.6f};

// Highest wheel speed the open-loop characterisation actually reached.
static const float RPM_MAX = 400.0f;

static const uint32_t PWM_FREQ_HZ = 20000;
static const uint8_t PWM_RES_BITS = 8;

// ---------------------------------------------------------------------------
// Types
// ---------------------------------------------------------------------------

enum MotorId { MOTOR_LEFT = 0, MOTOR_RIGHT = 1 };

struct WheelMeas {
  int32_t lastCount;
  uint32_t lastMicros;
  int32_t deltaCounts;  // counts in the most recent sample, for odometry
  float rpm;            // this sample only
  float rpmFilt;        // exponentially smoothed
  float mps;            // linear wheel speed from rpmFilt
};

WheelMeas wheel[2];

// ---------------------------------------------------------------------------
// Unit conversions (PDF p.21: omega = 2*pi*RPM/60, v = r*omega)
// ---------------------------------------------------------------------------

inline float rpmToRadPerSec(float rpm) { return 2.0f * PI * rpm / 60.0f; }
inline float radPerSecToRpm(float w) { return w * 60.0f / (2.0f * PI); }
inline float rpmToMps(float rpm) { return rpmToRadPerSec(rpm) * WHEEL_RADIUS_M; }
inline float mpsToRpm(float mps) { return radPerSecToRpm(mps / WHEEL_RADIUS_M); }

// ---------------------------------------------------------------------------
// Quadrature decoding: index = (previous state << 2) | current state,
// state = (A << 1) | B. Zero entries are no-change or an illegal transition.
// ---------------------------------------------------------------------------

static const int8_t QUAD_TABLE[16] = {
    0, -1, +1,  0,
   +1,  0,  0, -1,
   -1,  0,  0, +1,
    0, +1, -1,  0
};

volatile int32_t encCount[2] = {0, 0};
volatile uint8_t encState[2] = {0, 0};

void IRAM_ATTR leftEncoderISR() {
  uint8_t s = (digitalRead(ENC_L_C1) << 1) | digitalRead(ENC_L_C2);
  encCount[MOTOR_LEFT] += QUAD_TABLE[(encState[MOTOR_LEFT] << 2) | s];
  encState[MOTOR_LEFT] = s;
}

void IRAM_ATTR rightEncoderISR() {
  uint8_t s = (digitalRead(ENC_R_C1) << 1) | digitalRead(ENC_R_C2);
  encCount[MOTOR_RIGHT] += QUAD_TABLE[(encState[MOTOR_RIGHT] << 2) | s];
  encState[MOTOR_RIGHT] = s;
}

int32_t readEncoder(MotorId m) {
  noInterrupts();
  int32_t c = encCount[m];
  interrupts();
  return c * ENCODER_SIGN[m];
}

// ---------------------------------------------------------------------------
// Motor output
// ---------------------------------------------------------------------------

// pwm is signed: positive drives forward, negative reverse, 0 coasts.
void setMotorPwm(MotorId m, int pwm) {
  const int in1 = (m == MOTOR_LEFT) ? AIN1 : BIN1;
  const int in2 = (m == MOTOR_LEFT) ? AIN2 : BIN2;
  const int pwmPin = (m == MOTOR_LEFT) ? PWMA : PWMB;

  const int mag = constrain(abs(pwm), 0, 255);
  if (mag == 0) {
    analogWrite(pwmPin, 0);
    digitalWrite(in1, LOW);  // both low = coast, no braking transient
    digitalWrite(in2, LOW);
    return;
  }
  digitalWrite(in1, pwm > 0 ? LOW : HIGH);
  digitalWrite(in2, pwm > 0 ? HIGH : LOW);
  analogWrite(pwmPin, mag);
}

void motorsCoast() {
  setMotorPwm(MOTOR_LEFT, 0);
  setMotorPwm(MOTOR_RIGHT, 0);
}

// Open-loop prediction from the Activity 1 characteristic. Used as a
// feedforward term so the controllers only have to correct the residual.
int feedforwardPwm(MotorId m, float rpm) {
  const float mag = fabsf(rpm);
  if (mag < 0.5f) {
    return 0;
  }
  const int pwm = (int)lroundf(FF_SLOPE[m] * mag + FF_INTERCEPT[m]);
  return (rpm > 0.0f ? 1 : -1) * constrain(pwm, 0, 255);
}

// ---------------------------------------------------------------------------
// Measurement
// ---------------------------------------------------------------------------

// Call once per control cycle. Uses the true elapsed micros(), so a cycle that
// runs late still yields a correct speed rather than a wrong one.
// alpha is the exponential smoothing factor: 1.0 disables filtering.
void robotSample(float alpha) {
  const uint32_t now = micros();
  for (int i = 0; i < 2; i++) {
    const int32_t c = readEncoder((MotorId)i);
    const int32_t d = c - wheel[i].lastCount;
    const uint32_t dt = now - wheel[i].lastMicros;  // unsigned wrap is correct
    wheel[i].lastCount = c;
    wheel[i].lastMicros = now;
    wheel[i].deltaCounts = d;
    if (dt == 0) {
      continue;
    }
    const float countsPerSec = (float)d * 1e6f / (float)dt;
    wheel[i].rpm = countsPerSec * 60.0f / COUNTS_PER_OUTPUT_REV;
    wheel[i].rpmFilt += alpha * (wheel[i].rpm - wheel[i].rpmFilt);
    wheel[i].mps = rpmToMps(wheel[i].rpmFilt);
  }
}

void robotBaseBegin() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);

  pinMode(ENC_L_C1, INPUT_PULLUP);
  pinMode(ENC_L_C2, INPUT_PULLUP);
  pinMode(ENC_R_C1, INPUT_PULLUP);
  pinMode(ENC_R_C2, INPUT_PULLUP);

  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
#if defined(ESP_ARDUINO_VERSION) && \
    ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  analogWriteResolution(PWMA, PWM_RES_BITS);
  analogWriteResolution(PWMB, PWM_RES_BITS);
  analogWriteFrequency(PWMA, PWM_FREQ_HZ);
  analogWriteFrequency(PWMB, PWM_FREQ_HZ);
#endif

  encState[MOTOR_LEFT] = (digitalRead(ENC_L_C1) << 1) | digitalRead(ENC_L_C2);
  encState[MOTOR_RIGHT] = (digitalRead(ENC_R_C1) << 1) | digitalRead(ENC_R_C2);

  attachInterrupt(digitalPinToInterrupt(ENC_L_C1), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_C2), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C1), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C2), rightEncoderISR, CHANGE);

  const uint32_t now = micros();
  for (int i = 0; i < 2; i++) {
    wheel[i] = {readEncoder((MotorId)i), now, 0, 0.0f, 0.0f, 0.0f};
  }

  digitalWrite(STBY, HIGH);  // enable the driver; motors still coast at PWM 0
  motorsCoast();
}

#endif  // ROBOT_BASE_H
