/*
 * Day3Activity5.ino
 * Activity 5 - Step Response of the Wheel Velocity Controller  (lab PDF p.26)
 *
 * Commands a sequence of desired wheel velocities, records desired and actual
 * RPM at every control cycle, and streams the result as CSV. analyze_step.py
 * plots it and extracts rise time, settling time, overshoot and steady-state
 * error for each step.
 *
 * Board: ESP32-S3-DevKitC-1
 */

#include <Arduino.h>
#include "RobotBase.h"
#include "VelocityPI.h"

const uint32_t CONTROL_INTERVAL_MS = 20;   // 50 Hz - also the logging rate
const float FILTER_ALPHA = 0.30f;

// The step sequence. Held for STEP_HOLD_MS each, then repeats from the top.
// Starts and ends at zero so the fall response is captured too.
const float STEP_RPM[] = {0, 150, 0, 250, 100, 250, 0};
const int NUM_STEPS = sizeof(STEP_RPM) / sizeof(STEP_RPM[0]);
const uint32_t STEP_HOLD_MS = 2500;

VelocityPI ctrl[2];
float kp = 0.35f;
float ki = 2.50f;

bool running = false;
int stepIndex = 0;
uint32_t stepStartMs = 0;
uint32_t runStartMs = 0;
float setpointRpm = 0.0f;

char cmdBuf[24];
uint8_t cmdLen = 0;

void printCsvHeader() {
  Serial.println("t_ms,step_idx,setpoint_rpm,rpm_l,rpm_r,err_l,err_r,"
                 "pwm_l,pwm_r,kp,ki");
}

void printHelp() {
  Serial.println();
  Serial.println("commands (end with Enter):");
  Serial.println("  r        run the step sequence once (streams CSV)");
  Serial.println("  k<Kp>    proportional gain, e.g. k0.5");
  Serial.println("  i<Ki>    integral gain, e.g. i2.5  (i0 = pure P)");
  Serial.println("  s        stop immediately");
  Serial.println("  h        this help");
  Serial.println();
  Serial.printf("sequence:");
  for (int i = 0; i < NUM_STEPS; i++) {
    Serial.printf(" %.0f", STEP_RPM[i]);
  }
  Serial.printf(" rpm, %.1f s each (%.1f s total)\n",
                STEP_HOLD_MS / 1000.0f, NUM_STEPS * STEP_HOLD_MS / 1000.0f);
}

void startRun() {
  piInit(ctrl[MOTOR_LEFT], kp, ki);
  piInit(ctrl[MOTOR_RIGHT], kp, ki);
  stepIndex = 0;
  setpointRpm = STEP_RPM[0];
  runStartMs = millis();
  stepStartMs = runStartMs;
  running = true;
  Serial.println();
  printCsvHeader();
}

void stopRun() {
  running = false;
  setpointRpm = 0.0f;
  motorsCoast();
  Serial.println("# run stopped");
}

void handleCommand(const char *cmd) {
  const float arg = atof(cmd + 1);
  switch (cmd[0]) {
    case 'r': startRun(); break;
    case 's': stopRun(); break;
    case 'k':
      kp = max(0.0f, arg);
      Serial.printf("# Kp = %.3f\n", kp);
      break;
    case 'i':
      ki = max(0.0f, arg);
      Serial.printf("# Ki = %.3f\n", ki);
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

void setup() {
  Serial.begin(115200);
  delay(500);
  robotBaseBegin();
  piInit(ctrl[MOTOR_LEFT], kp, ki);
  piInit(ctrl[MOTOR_RIGHT], kp, ki);

  Serial.println();
  Serial.println("=== Activity 5 - Step Response of the Wheel Velocity Controller ===");
  Serial.printf("control loop %u ms (%.0f Hz), one CSV row per cycle\n",
                CONTROL_INTERVAL_MS, 1000.0f / CONTROL_INTERVAL_MS);
  Serial.println("Lift the chassis so the wheels spin free.");
  printHelp();
}

void loop() {
  static uint32_t nextControl = 0;
  const uint32_t now = millis();

  pollSerial();

  if ((int32_t)(now - nextControl) < 0) {
    return;
  }
  nextControl = (nextControl == 0 ? now : nextControl) + CONTROL_INTERVAL_MS;
  const float dt = CONTROL_INTERVAL_MS / 1000.0f;

  robotSample(FILTER_ALPHA);

  if (!running) {
    motorsCoast();
    return;
  }

  if (now - stepStartMs >= STEP_HOLD_MS) {
    stepIndex++;
    if (stepIndex >= NUM_STEPS) {
      motorsCoast();
      running = false;
      Serial.println("# done. Send 'r' to run again.");
      return;
    }
    stepStartMs = now;
    setpointRpm = STEP_RPM[stepIndex];
  }

  if (setpointRpm == 0.0f) {
    // Coast rather than commanding zero through the controller, so the fall
    // response measured is the machine's own deceleration.
    motorsCoast();
    piReset(ctrl[MOTOR_LEFT]);
    piReset(ctrl[MOTOR_RIGHT]);
    ctrl[MOTOR_LEFT].error = -wheel[MOTOR_LEFT].rpmFilt;
    ctrl[MOTOR_RIGHT].error = -wheel[MOTOR_RIGHT].rpmFilt;
  } else {
    setMotorPwm(MOTOR_LEFT,
                piUpdate(ctrl[MOTOR_LEFT], MOTOR_LEFT, setpointRpm,
                         wheel[MOTOR_LEFT].rpmFilt, dt));
    setMotorPwm(MOTOR_RIGHT,
                piUpdate(ctrl[MOTOR_RIGHT], MOTOR_RIGHT, setpointRpm,
                         wheel[MOTOR_RIGHT].rpmFilt, dt));
  }

  Serial.printf("%lu,%d,%.1f,%.2f,%.2f,%.2f,%.2f,%d,%d,%.3f,%.3f\n",
                now - runStartMs, stepIndex, setpointRpm,
                wheel[MOTOR_LEFT].rpmFilt, wheel[MOTOR_RIGHT].rpmFilt,
                ctrl[MOTOR_LEFT].error, ctrl[MOTOR_RIGHT].error,
                ctrl[MOTOR_LEFT].output, ctrl[MOTOR_RIGHT].output, kp, ki);
}
