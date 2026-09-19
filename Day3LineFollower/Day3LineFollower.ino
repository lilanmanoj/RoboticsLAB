/*
 * Day3LineFollower.ino
 * Activity 1 (line-following section, lab PDF p.38)
 *   "Use a 5-bit IR sensor array to design a line following robot that follows
 *    a given line with a PID controller."
 *
 * Two nested loops:
 *   outer - PID on the line position error from the 5-bit MD0482 array,
 *           producing a differential RPM correction
 *   inner - the Activity 4 PI wheel velocity controllers, which execute
 *           base +/- correction on each wheel
 *
 * Running the steering output through velocity control rather than straight to
 * PWM means the robot follows the line the same way on a fresh battery as on a
 * flat one, and the 3.3% left/right gain mismatch from Activity 1 stops
 * biasing the steering.
 *
 * Board: ESP32-S3-DevKitC-1
 */

#include <Arduino.h>
#include "RobotBase.h"
#include "VelocityPI.h"

// ---------------------------------------------------------------------------
// 5-bit IR array (MD0482) - D0..D4, leftmost to rightmost
//
// THESE PINS ARE NOT IN THE REPOSITORY README. They are free GPIOs on the
// ESP32-S3-DevKitC-1 chosen to avoid the strapping pins (0/3/45/46), the USB
// D+/D- pair (19/20) and everything already used by the drivetrain. Change
// them to match how you actually wire the module.
// ---------------------------------------------------------------------------
const int IR_PIN[5] = {6, 7, 15, 16, 17};

// Module output is digital TTL and active low: a 0 bit means that sensor is
// over the line. PDF p.36: 11011 = centred, 01111 = line off to the left.
// Set false if your module reads the other way round (white line on black).
const bool IR_ACTIVE_LOW = true;

// Sensor positions used to weight the error, in "sensor widths" from centre.
const float IR_WEIGHT[5] = {-2.0f, -1.0f, 0.0f, +1.0f, +2.0f};

// ---------------------------------------------------------------------------
// Control
// ---------------------------------------------------------------------------

const uint32_t CONTROL_INTERVAL_MS = 20;   // 50 Hz, outer and inner together
const uint32_t DISPLAY_INTERVAL_MS = 250;
const float FILTER_ALPHA = 0.30f;

// Steering PID. Output is a differential RPM correction: +corr to the right
// wheel, -corr to the left, which turns the robot left.
float sKp = 28.0f;
float sKi = 0.0f;      // usually left at 0 - a line follower has no standing
                       // offset to integrate away, and Ki mostly adds wobble
float sKd = 12.0f;

float baseRpm = 120.0f;      // cruise speed of both wheels
float maxCorrRpm = 120.0f;   // ceiling on the differential correction

// Inner velocity loops.
float kp = 0.35f;
float ki = 2.50f;
VelocityPI ctrl[2];

// Steering state.
float lineError = 0.0f;
float lastLineError = 0.0f;
float errorIntegral = 0.0f;
float derivative = 0.0f;
float correction = 0.0f;
uint8_t sensorBits = 0;      // bit i set = sensor i sees the line
int sensorsOnLine = 0;

// Last direction the line was seen, used to search when it is lost entirely.
float lastSeenSign = 0.0f;
uint32_t lostSinceMs = 0;
const uint32_t LOST_GIVEUP_MS = 1200;

bool running = false;
bool logging = false;

float wheelSetRpm[2] = {0.0f, 0.0f};

char cmdBuf[32];
uint8_t cmdLen = 0;

// ---------------------------------------------------------------------------

void readSensors() {
  sensorBits = 0;
  sensorsOnLine = 0;
  for (int i = 0; i < 5; i++) {
    const int raw = digitalRead(IR_PIN[i]);
    const bool onLine = IR_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);
    if (onLine) {
      sensorBits |= (1 << i);
      sensorsOnLine++;
    }
  }
}

/*
 * Weighted-average line position.
 *
 * Averaging the positions of every sensor that sees the line gives a
 * continuous error even though the sensors are discrete - with two adjacent
 * sensors triggered the error lands halfway between them. A pure
 * highest-priority-sensor lookup would quantise the error to five values and
 * make the derivative term useless.
 *
 * Returns false when no sensor sees the line at all.
 */
bool computeLineError(float &err) {
  if (sensorsOnLine == 0) {
    return false;
  }
  float sum = 0.0f;
  for (int i = 0; i < 5; i++) {
    if (sensorBits & (1 << i)) {
      sum += IR_WEIGHT[i];
    }
  }
  err = sum / sensorsOnLine;
  return true;
}

void steeringStep(float dt) {
  float err;
  const bool seen = computeLineError(err);

  if (seen) {
    lineError = err;
    lastSeenSign = (err > 0.0f) ? 1.0f : (err < 0.0f ? -1.0f : lastSeenSign);
    lostSinceMs = 0;
  } else {
    // Line lost: keep steering hard the way it was last seen, so the robot
    // sweeps back onto it rather than driving straight off the track.
    if (lostSinceMs == 0) {
      lostSinceMs = millis();
    }
    lineError = lastSeenSign * 2.5f;
  }

  errorIntegral += lineError * dt;
  errorIntegral = constrain(errorIntegral, -50.0f, 50.0f);
  derivative = (lineError - lastLineError) / dt;
  lastLineError = lineError;

  correction = sKp * lineError + sKi * errorIntegral + sKd * derivative;
  correction = constrain(correction, -maxCorrRpm, maxCorrRpm);
}

void computeWheelSetpoints() {
  // Positive lineError means the line is to the RIGHT of centre, so the robot
  // must turn right: speed up the left wheel, slow the right one.
  float l = baseRpm + correction;
  float r = baseRpm - correction;

  const float peak = max(fabsf(l), fabsf(r));
  if (peak > RPM_MAX) {
    const float scale = RPM_MAX / peak;
    l *= scale;
    r *= scale;
  }
  wheelSetRpm[MOTOR_LEFT] = l;
  wheelSetRpm[MOTOR_RIGHT] = r;
}

void stopRobot() {
  running = false;
  wheelSetRpm[0] = wheelSetRpm[1] = 0.0f;
  errorIntegral = 0.0f;
  correction = 0.0f;
  piReset(ctrl[0]);
  piReset(ctrl[1]);
  motorsCoast();
}

void printHelp() {
  Serial.println();
  Serial.println("commands (end with Enter):");
  Serial.println("  G          go / start following");
  Serial.println("  s          stop");
  Serial.println("  b<rpm>     base cruise speed,   e.g. b120");
  Serial.println("  p<Kp>      steering Kp,         e.g. p28");
  Serial.println("  i<Ki>      steering Ki,         e.g. i0");
  Serial.println("  d<Kd>      steering Kd,         e.g. d12");
  Serial.println("  m<rpm>     max differential correction");
  Serial.println("  P<Kp>/I<Ki>  inner wheel velocity gains");
  Serial.println("  t          sensor test - print the 5 bits only");
  Serial.println("  g          toggle CSV logging");
  Serial.println("  h          this help");
  Serial.println();
}

void printCsvHeader() {
  Serial.println("t_ms,bits,on_line,line_err,integral,deriv,correction,"
                 "set_rpm_l,set_rpm_r,rpm_l,rpm_r,pwm_l,pwm_r");
}

void printBits() {
  for (int i = 0; i < 5; i++) {
    // Print in the same convention as the lab sheet: 0 = sensor on the line.
    Serial.print((sensorBits & (1 << i)) ? '0' : '1');
  }
}

bool sensorTest = false;

void handleCommand(const char *cmd) {
  const float arg = atof(cmd + 1);
  switch (cmd[0]) {
    case 'G':
      piReset(ctrl[0]);
      piReset(ctrl[1]);
      errorIntegral = 0.0f;
      lastLineError = 0.0f;
      running = true;
      sensorTest = false;
      break;
    case 's': stopRobot(); break;
    case 'b': baseRpm = constrain(arg, 0.0f, RPM_MAX); break;
    case 'p': sKp = max(0.0f, arg); break;
    case 'i': sKi = max(0.0f, arg); errorIntegral = 0.0f; break;
    case 'd': sKd = max(0.0f, arg); break;
    case 'm': maxCorrRpm = max(0.0f, arg); break;
    case 'P': kp = max(0.0f, arg); ctrl[0].kp = ctrl[1].kp = kp; break;
    case 'I': ki = max(0.0f, arg); ctrl[0].ki = ctrl[1].ki = ki; break;
    case 't':
      sensorTest = !sensorTest;
      if (sensorTest) {
        stopRobot();
        Serial.println("\nsensor test - move the array over the line");
      }
      break;
    case 'g':
      logging = !logging;
      Serial.println();
      if (logging) printCsvHeader();
      break;
    default: printHelp(); break;
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

void display() {
  if (sensorTest) {
    Serial.printf("\033[1A sensors ");
    printBits();
    Serial.printf("   on line %d   error %6.2f\033[K\n",
                  sensorsOnLine, lineError);
    return;
  }
  Serial.printf("\033[5A");
  Serial.printf(" %-7s sensors ", running ? "RUN" : "idle");
  printBits();
  Serial.printf("  on line %d  base %.0f rpm\033[K\n", sensorsOnLine, baseRpm);
  Serial.printf(" steering  err %6.2f  I %7.2f  D %8.2f  -> corr %7.1f rpm\033[K\n",
                lineError, errorIntegral, derivative, correction);
  Serial.printf(" gains     Kp %.1f  Ki %.2f  Kd %.1f\033[K\n", sKp, sKi, sKd);
  Serial.printf(" LEFT   set %7.1f  act %7.1f rpm  pwm %5d\033[K\n",
                wheelSetRpm[MOTOR_LEFT], wheel[MOTOR_LEFT].rpmFilt,
                ctrl[MOTOR_LEFT].output);
  Serial.printf(" RIGHT  set %7.1f  act %7.1f rpm  pwm %5d\033[K\n",
                wheelSetRpm[MOTOR_RIGHT], wheel[MOTOR_RIGHT].rpmFilt,
                ctrl[MOTOR_RIGHT].output);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  for (int i = 0; i < 5; i++) {
    pinMode(IR_PIN[i], INPUT);
  }
  robotBaseBegin();
  piInit(ctrl[MOTOR_LEFT], kp, ki);
  piInit(ctrl[MOTOR_RIGHT], kp, ki);

  Serial.println();
  Serial.println("=== Line Following Robot - 5-bit IR array + PID ===");
  Serial.printf("IR pins D0..D4 = %d,%d,%d,%d,%d  (active %s)\n",
                IR_PIN[0], IR_PIN[1], IR_PIN[2], IR_PIN[3], IR_PIN[4],
                IR_ACTIVE_LOW ? "LOW" : "HIGH");
  Serial.println("These pins are a guess - check them against your wiring.");
  Serial.println("Run 't' first to verify the sensors before sending 'G'.");
  printHelp();
  Serial.println("\n\n\n\n\n");
}

void loop() {
  static uint32_t nextControl = 0;
  static uint32_t nextDisplay = 0;
  const uint32_t now = millis();

  pollSerial();

  if ((int32_t)(now - nextControl) >= 0) {
    nextControl = (nextControl == 0 ? now : nextControl) + CONTROL_INTERVAL_MS;
    const float dt = CONTROL_INTERVAL_MS / 1000.0f;

    robotSample(FILTER_ALPHA);
    readSensors();

    if (running) {
      steeringStep(dt);
      computeWheelSetpoints();

      // Give up rather than drive blind if the line has been gone a while.
      if (lostSinceMs != 0 && now - lostSinceMs > LOST_GIVEUP_MS) {
        Serial.println("\n# line lost - stopping");
        stopRobot();
      }
    } else {
      wheelSetRpm[0] = wheelSetRpm[1] = 0.0f;
      if (sensorTest) {
        computeLineError(lineError);
      }
    }

    for (int i = 0; i < 2; i++) {
      const MotorId m = (MotorId)i;
      if (fabsf(wheelSetRpm[m]) < 0.5f) {
        setMotorPwm(m, 0);
        piReset(ctrl[m]);
        ctrl[m].error = -wheel[m].rpmFilt;
      } else {
        setMotorPwm(m, piUpdate(ctrl[m], m, wheelSetRpm[m],
                                wheel[m].rpmFilt, dt));
      }
    }

    if (logging) {
      Serial.printf("%lu,%d,%d,%.3f,%.3f,%.2f,%.1f,%.1f,%.1f,%.2f,%.2f,%d,%d\n",
                    now, sensorBits, sensorsOnLine, lineError, errorIntegral,
                    derivative, correction,
                    wheelSetRpm[MOTOR_LEFT], wheelSetRpm[MOTOR_RIGHT],
                    wheel[MOTOR_LEFT].rpmFilt, wheel[MOTOR_RIGHT].rpmFilt,
                    ctrl[MOTOR_LEFT].output, ctrl[MOTOR_RIGHT].output);
    }
  }

  if (!logging && (int32_t)(now - nextDisplay) >= 0) {
    nextDisplay = (nextDisplay == 0 ? now : nextDisplay) + DISPLAY_INTERVAL_MS;
    display();
  }
}
