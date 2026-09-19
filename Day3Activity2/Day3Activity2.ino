/*
 * Day3Activity2.ino
 * Activity 2 - Measure and Display Actual Velocity
 *
 * Select a desired wheel velocity over Serial, drive the wheels open-loop
 * using the feedforward model measured in Activity 1, sample the encoders at a
 * fixed interval, and display target / actual / error in real time.
 *
 * This is still OPEN LOOP: the commanded PWM never reacts to the measured
 * speed. The error column is the whole point of the activity - it shows how
 * far the feedforward model drifts from reality, which is what motivates
 * closing the loop.
 *
 * Board: ESP32-S3-DevKitC-1
 * Driver: TB6612FNG   Motors: N20 6V 300RPM w/ hall quadrature encoder
 */

#include <Arduino.h>

// Declared before any function: the Arduino preprocessor injects generated
// prototypes ahead of the first definition, so these must already be visible.
enum MotorId { MOTOR_LEFT = 0, MOTOR_RIGHT = 1 };

struct Wheel {
  int32_t lastCount;
  uint32_t lastMicros;
  float targetRpm;   // commanded setpoint
  float rpmRaw;      // this sample only
  float rpmFilt;     // exponentially smoothed
  int pwm;           // what the feedforward actually applied
};

// ---------------------------------------------------------------------------
// Pin map
//
// NOTE: the motor-to-encoder pairing is crossed relative to ../README.md.
// Verified on the bench: TB6612FNG channel A turns the encoder on GPIO 36/37,
// channel B turns the encoder on GPIO 38/39. Pins below follow the hardware.
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
// Calibration  --  CHECK BOTH OF THESE BEFORE TRUSTING THE NUMBERS
// ---------------------------------------------------------------------------

// Same constant as Activity 1: 7 PPR/channel x4 quadrature x 30:1 gearbox.
// Activity 1 produced ~420 RPM at full duty on a 5 V rail, which is high for a
// 300 RPM/6 V motor - so this value is probably wrong. Calibrate by marking a
// wheel and turning it exactly 10 revolutions: counts/10 is the true value.
const float COUNTS_PER_OUTPUT_REV = 840.0f;

// ASSUMPTION: 43 mm wheels (a common N20 size). This is not in the repository
// README, so measure your actual wheel and correct it - every m/s figure this
// sketch prints scales directly with it. RPM figures are unaffected.
const float WHEEL_DIAMETER_MM = 43.0f;

// Encoder polarity. Both motors' terminals are wired crossed the same way, so
// "forward" drive counts up on A and down on B.
const int LEFT_ENCODER_SIGN = +1;
const int RIGHT_ENCODER_SIGN = -1;

// ---------------------------------------------------------------------------
// Feedforward model from Activity 1:  pwm = FF_SLOPE * rpm + FF_INTERCEPT
//
// Measured 2026-08-30 (day3_activity1.csv):
//   left  gain 1.710 RPM/PWM, fitted dead-band  9.6  ->  pwm = 0.585*w +  9.6
//   right gain 1.654 RPM/PWM, fitted dead-band 10.6  ->  pwm = 0.605*w + 10.6
//
// These are tied to COUNTS_PER_OUTPUT_REV above. If you recalibrate that,
// re-run Activity 1 and paste the new numbers here.
//
// The fitted dead-band understates reality at low speed: the left wheel does
// not actually break away until PWM ~50 and stalls below ~75 on the way down.
// Expect large errors below roughly 90 RPM on the left wheel.
// ---------------------------------------------------------------------------
const float FF_SLOPE[2] = {0.585f, 0.605f};      // PWM per RPM
const float FF_INTERCEPT[2] = {9.6f, 10.6f};     // PWM at zero speed

// ---------------------------------------------------------------------------
// Sampling and display
// ---------------------------------------------------------------------------

// Encoders are sampled fast and steadily; the terminal is repainted more
// slowly so the display stays readable without slowing the measurement.
const uint32_t SAMPLE_INTERVAL_MS = 50;    // 20 Hz measurement rate
const uint32_t DISPLAY_INTERVAL_MS = 250;  // 4 Hz repaint

// Exponential moving average on the measured speed. 1.0 disables filtering.
// Encoder quantisation at 50 ms is a few RPM; this trades lag for steadiness.
float filterAlpha = 0.30f;

// Repaint the table in place using ANSI cursor movement. Set false (or press
// 't' for CSV mode) if your terminal shows escape codes as garbage, or when
// piping the output to a file.
bool ansiRepaint = true;
const int DISPLAY_LINES = 5;

bool csvMode = false;

const uint32_t PWM_FREQ_HZ = 20000;
const uint8_t PWM_RES_BITS = 8;
const unsigned long SERIAL_BAUD = 115200;

const float WHEEL_CIRCUM_M = WHEEL_DIAMETER_MM * 0.001f * PI;

// ---------------------------------------------------------------------------
// Quadrature decoding
// ---------------------------------------------------------------------------

static const int8_t QUAD_TABLE[16] = {
    0, -1, +1,  0,
   +1,  0,  0, -1,
   -1,  0,  0, +1,
    0, +1, -1,  0
};

volatile int32_t leftCount = 0;
volatile int32_t rightCount = 0;
volatile uint8_t leftState = 0;
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

int32_t readCount(MotorId motor) {
  noInterrupts();
  int32_t c = (motor == MOTOR_LEFT) ? leftCount : rightCount;
  interrupts();
  return c * ((motor == MOTOR_LEFT) ? LEFT_ENCODER_SIGN : RIGHT_ENCODER_SIGN);
}

// ---------------------------------------------------------------------------
// Motor output
// ---------------------------------------------------------------------------

Wheel wheels[2];
bool rawPwmMode = false;  // 'p' command drives PWM directly, no feedforward

void applyDrive(MotorId motor, bool forward, int pwm) {
  const int in1 = (motor == MOTOR_LEFT) ? AIN1 : BIN1;
  const int in2 = (motor == MOTOR_LEFT) ? AIN2 : BIN2;
  const int pwmPin = (motor == MOTOR_LEFT) ? PWMA : PWMB;

  if (pwm <= 0) {
    analogWrite(pwmPin, 0);
    digitalWrite(in1, LOW);  // both low = coast
    digitalWrite(in2, LOW);
    return;
  }
  digitalWrite(in1, forward ? LOW : HIGH);
  digitalWrite(in2, forward ? HIGH : LOW);
  analogWrite(pwmPin, pwm);
}

// Inverse of the Activity 1 characteristic: the PWM that should produce this
// speed. Open loop - this is a prediction, never a correction.
int pwmForRpm(MotorId motor, float rpm) {
  const float mag = fabsf(rpm);
  if (mag < 0.5f) {
    return 0;
  }
  const float pwm = FF_SLOPE[motor] * mag + FF_INTERCEPT[motor];
  return constrain((int)lroundf(pwm), 0, 255);
}

void updateOutputs() {
  for (int i = 0; i < 2; i++) {
    const MotorId m = (MotorId)i;
    if (!rawPwmMode) {
      wheels[i].pwm = pwmForRpm(m, wheels[i].targetRpm);
    }
    applyDrive(m, wheels[i].targetRpm >= 0.0f, wheels[i].pwm);
  }
  const bool anyDrive = wheels[0].pwm > 0 || wheels[1].pwm > 0;
  digitalWrite(STBY, anyDrive ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
// Measurement
// ---------------------------------------------------------------------------

float rpmToMps(float rpm) { return rpm / 60.0f * WHEEL_CIRCUM_M; }
float mpsToRpm(float mps) { return mps * 60.0f / WHEEL_CIRCUM_M; }

void sampleWheel(MotorId motor) {
  Wheel &w = wheels[motor];
  const int32_t count = readCount(motor);
  const uint32_t now = micros();

  const int32_t dCounts = count - w.lastCount;
  const uint32_t dMicros = now - w.lastMicros;  // unsigned math wraps correctly
  w.lastCount = count;
  w.lastMicros = now;

  if (dMicros == 0) {
    return;
  }
  const float countsPerSec = (float)dCounts * 1e6f / (float)dMicros;
  w.rpmRaw = countsPerSec * 60.0f / COUNTS_PER_OUTPUT_REV;
  w.rpmFilt += filterAlpha * (w.rpmRaw - w.rpmFilt);
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------

void printHelp() {
  Serial.println();
  Serial.println("commands (end with Enter):");
  Serial.println("  v<rpm>   set both wheels, e.g. v150   (negative = reverse)");
  Serial.println("  m<m/s>   set both wheels in m/s, e.g. m0.35");
  Serial.println("  l<rpm>   set LEFT wheel only");
  Serial.println("  r<rpm>   set RIGHT wheel only");
  Serial.println("  p<pwm>   raw PWM, bypasses the feedforward model");
  Serial.println("  s        stop");
  Serial.println("  f<0..1>  filter strength (1 = off), e.g. f0.3");
  Serial.println("  t        toggle live table / CSV log output");
  Serial.println("  h        this help");
  Serial.println();
}

void printBanner() {
  Serial.println();
  Serial.println("=== Day 3 - Activity 2: Measure and Display Actual Velocity ===");
  Serial.printf("counts_per_output_rev = %.1f   (calibrate: 10 hand turns)\n",
                COUNTS_PER_OUTPUT_REV);
  Serial.printf("wheel diameter        = %.1f mm  -> circumference %.4f m\n",
                WHEEL_DIAMETER_MM, WHEEL_CIRCUM_M);
  Serial.printf("sample interval       = %u ms (%.0f Hz)\n",
                SAMPLE_INTERVAL_MS, 1000.0f / SAMPLE_INTERVAL_MS);
  Serial.printf("feedforward LEFT      = %.3f*rpm + %.1f\n",
                FF_SLOPE[0], FF_INTERCEPT[0]);
  Serial.printf("feedforward RIGHT     = %.3f*rpm + %.1f\n",
                FF_SLOPE[1], FF_INTERCEPT[1]);
  Serial.println("OPEN LOOP - the error column is measured, never corrected.");
  Serial.println("Lift the chassis so the wheels spin free.");
}

void printCsvHeader() {
  Serial.println("t_ms,target_rpm_l,rpm_l,err_rpm_l,mps_l,pwm_l,"
                 "target_rpm_r,rpm_r,err_rpm_r,mps_r,pwm_r");
}

void displayCsv() {
  const Wheel &l = wheels[MOTOR_LEFT];
  const Wheel &r = wheels[MOTOR_RIGHT];
  Serial.printf("%lu,%.2f,%.2f,%.2f,%.4f,%d,%.2f,%.2f,%.2f,%.4f,%d\n",
                millis(),
                l.targetRpm, l.rpmFilt, l.targetRpm - l.rpmFilt,
                rpmToMps(l.rpmFilt), l.pwm,
                r.targetRpm, r.rpmFilt, r.targetRpm - r.rpmFilt,
                rpmToMps(r.rpmFilt), r.pwm);
}

void displayRow(const char *name, const Wheel &w) {
  const float errRpm = w.targetRpm - w.rpmFilt;
  const float targetMps = rpmToMps(w.targetRpm);
  const float actualMps = rpmToMps(w.rpmFilt);
  Serial.printf(" %-5s |%8.1f %8.1f %8.1f |%8.3f %8.3f %8.3f |%5d\033[K\n",
                name, w.targetRpm, w.rpmFilt, errRpm,
                targetMps, actualMps, targetMps - actualMps, w.pwm);
}

void displayTable() {
  static bool painted = false;
  if (ansiRepaint && painted) {
    Serial.printf("\033[%dA", DISPLAY_LINES);
  }
  painted = true;

  Serial.printf(" wheel |  tgt rpm  act rpm  err rpm |  tgt m/s  act m/s  err m/s"
                " |  pwm\033[K\n");
  Serial.printf(" ------+---------------------------+---------------------------"
                "+-----\033[K\n");
  displayRow("LEFT", wheels[MOTOR_LEFT]);
  displayRow("RIGHT", wheels[MOTOR_RIGHT]);
  Serial.printf(" filter alpha %.2f | %s | 't' for CSV, 'h' for help\033[K\n",
                filterAlpha, rawPwmMode ? "RAW PWM" : "feedforward");
}

// ---------------------------------------------------------------------------
// Serial commands (non-blocking, so the sample interval never slips)
// ---------------------------------------------------------------------------

char cmdBuf[24];
uint8_t cmdLen = 0;

void setTargetRpm(float rpm, bool left, bool right) {
  rawPwmMode = false;
  if (left) wheels[MOTOR_LEFT].targetRpm = rpm;
  if (right) wheels[MOTOR_RIGHT].targetRpm = rpm;
  updateOutputs();
}

void handleCommand(const char *cmd) {
  const char c = cmd[0];
  const float arg = atof(cmd + 1);

  switch (c) {
    case 'v': setTargetRpm(arg, true, true); break;
    case 'm': setTargetRpm(mpsToRpm(arg), true, true); break;
    case 'l': setTargetRpm(arg, true, false); break;
    case 'r': setTargetRpm(arg, false, true); break;
    case 'p':
      rawPwmMode = true;
      wheels[MOTOR_LEFT].pwm = constrain((int)arg, 0, 255);
      wheels[MOTOR_RIGHT].pwm = constrain((int)arg, 0, 255);
      wheels[MOTOR_LEFT].targetRpm = 0.0f;
      wheels[MOTOR_RIGHT].targetRpm = 0.0f;
      updateOutputs();
      break;
    case 's':
      rawPwmMode = false;
      wheels[MOTOR_LEFT].targetRpm = 0.0f;
      wheels[MOTOR_RIGHT].targetRpm = 0.0f;
      updateOutputs();
      break;
    case 'f':
      filterAlpha = constrain(arg, 0.01f, 1.0f);
      break;
    case 't':
      csvMode = !csvMode;
      Serial.println();
      if (csvMode) printCsvHeader();
      break;
    default:
      printHelp();
      break;
  }
}

void pollSerial() {
  while (Serial.available()) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        handleCommand(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
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

  leftState = (digitalRead(ENC_L_C1) << 1) | digitalRead(ENC_L_C2);
  rightState = (digitalRead(ENC_R_C1) << 1) | digitalRead(ENC_R_C2);

  attachInterrupt(digitalPinToInterrupt(ENC_L_C1), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_C2), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C1), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_C2), rightEncoderISR, CHANGE);

  const uint32_t now = micros();
  for (int i = 0; i < 2; i++) {
    wheels[i] = {readCount((MotorId)i), now, 0.0f, 0.0f, 0.0f, 0};
  }

  printBanner();
  printHelp();
}

void loop() {
  static uint32_t nextSample = 0;
  static uint32_t nextDisplay = 0;
  const uint32_t now = millis();

  pollSerial();

  // Fixed sampling interval: the deadline advances by a constant step rather
  // than from "now", so display work and serial traffic cannot make it drift.
  if ((int32_t)(now - nextSample) >= 0) {
    nextSample = (nextSample == 0 ? now : nextSample) + SAMPLE_INTERVAL_MS;
    sampleWheel(MOTOR_LEFT);
    sampleWheel(MOTOR_RIGHT);
    if (csvMode) {
      displayCsv();
    }
  }

  if (!csvMode && (int32_t)(now - nextDisplay) >= 0) {
    nextDisplay = (nextDisplay == 0 ? now : nextDisplay) + DISPLAY_INTERVAL_MS;
    displayTable();
  }
}
