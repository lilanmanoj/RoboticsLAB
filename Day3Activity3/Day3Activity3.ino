/*
 * Day3Activity3.ino
 * Activity 3 - Proportional Velocity Control  (lab PDF p.24)
 *
 *     e(t) = wd - w(t)
 *     u(t) = Kp e(t)          u is the PWM command
 *
 * Closes the loop on wheel speed with a pure proportional controller, so the
 * characteristic weakness of P control is visible: a nonzero steady-state
 * error that shrinks but never vanishes as Kp rises, and overshoot/oscillation
 * once Kp gets large.
 *
 * Board: ESP32-S3-DevKitC-1
 */

#include <Arduino.h>
#include "RobotBase.h"

const uint32_t CONTROL_INTERVAL_MS = 20;   // 50 Hz control loop
const uint32_t DISPLAY_INTERVAL_MS = 250;
const float FILTER_ALPHA = 0.30f;

// Start small. With feedforward off, Kp alone must build the whole PWM, so a
// useful value is roughly 1/gain from Activity 1 (~0.6 PWM per RPM).
float kp = 0.35f;

// Pure P as written in the PDF has no feedforward. Turn it on with 'w' to see
// the steady-state error collapse - that contrast is the lesson of the
// activity, so run it both ways.
bool useFeedforward = false;

float setpointRpm = 0.0f;
float lastError[2] = {0.0f, 0.0f};
int lastOutput[2] = {0, 0};
bool logging = false;

char cmdBuf[24];
uint8_t cmdLen = 0;

// u(t) = Kp e(t), optionally on top of the open-loop prediction.
int controlStep(MotorId m) {
  const float e = setpointRpm - wheel[m].rpmFilt;
  lastError[m] = e;

  const float ff = useFeedforward ? (float)feedforwardPwm(m, setpointRpm) : 0.0f;
  const float u = ff + kp * e;

  lastOutput[m] = constrain((int)lroundf(u), -255, 255);
  return lastOutput[m];
}

void printHelp() {
  Serial.println();
  Serial.println("commands (end with Enter):");
  Serial.println("  v<rpm>   desired wheel velocity, e.g. v150");
  Serial.println("  k<Kp>    proportional gain, e.g. k0.8");
  Serial.println("  w        toggle feedforward (pure P vs P + feedforward)");
  Serial.println("  s        stop");
  Serial.println("  g        toggle CSV logging of every control cycle");
  Serial.println("  h        this help");
  Serial.println();
}

void printBanner() {
  Serial.println();
  Serial.println("=== Activity 3 - Proportional Velocity Control ===");
  Serial.println("u(t) = Kp * e(t),  e(t) = desired RPM - measured RPM");
  Serial.printf("control loop %u ms (%.0f Hz), filter alpha %.2f\n",
                CONTROL_INTERVAL_MS, 1000.0f / CONTROL_INTERVAL_MS,
                FILTER_ALPHA);
  Serial.println("Lift the chassis so the wheels spin free.");
}

void printCsvHeader() {
  Serial.println("t_ms,kp,setpoint_rpm,rpm_l,err_l,pwm_l,rpm_r,err_r,pwm_r");
}

void handleCommand(const char *cmd) {
  const float arg = atof(cmd + 1);
  switch (cmd[0]) {
    case 'v':
      setpointRpm = constrain(arg, -RPM_MAX, RPM_MAX);
      break;
    case 'k':
      kp = max(0.0f, arg);
      Serial.printf("\nKp = %.3f\n", kp);
      break;
    case 'w':
      useFeedforward = !useFeedforward;
      Serial.printf("\nfeedforward %s\n", useFeedforward ? "ON" : "OFF");
      break;
    case 's':
      setpointRpm = 0.0f;
      motorsCoast();
      break;
    case 'g':
      logging = !logging;
      Serial.println();
      if (logging) printCsvHeader();
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

void display() {
  Serial.printf("\033[4A");
  Serial.printf(" Kp %.3f  feedforward %-3s  setpoint %6.1f rpm\033[K\n",
                kp, useFeedforward ? "ON" : "OFF", setpointRpm);
  Serial.printf(" wheel |  set rpm  act rpm  err rpm |   pwm\033[K\n");
  Serial.printf(" LEFT  |%9.1f%9.1f%9.1f |%6d\033[K\n",
                setpointRpm, wheel[MOTOR_LEFT].rpmFilt, lastError[MOTOR_LEFT],
                lastOutput[MOTOR_LEFT]);
  Serial.printf(" RIGHT |%9.1f%9.1f%9.1f |%6d\033[K\n",
                setpointRpm, wheel[MOTOR_RIGHT].rpmFilt, lastError[MOTOR_RIGHT],
                lastOutput[MOTOR_RIGHT]);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  robotBaseBegin();
  printBanner();
  printHelp();
  Serial.println("\n\n\n");  // reserve the 4 lines the display repaints
}

void loop() {
  static uint32_t nextControl = 0;
  static uint32_t nextDisplay = 0;
  const uint32_t now = millis();

  pollSerial();

  if ((int32_t)(now - nextControl) >= 0) {
    nextControl = (nextControl == 0 ? now : nextControl) + CONTROL_INTERVAL_MS;

    robotSample(FILTER_ALPHA);

    if (setpointRpm == 0.0f) {
      motorsCoast();
      lastOutput[0] = lastOutput[1] = 0;
      lastError[0] = -wheel[0].rpmFilt;
      lastError[1] = -wheel[1].rpmFilt;
    } else {
      setMotorPwm(MOTOR_LEFT, controlStep(MOTOR_LEFT));
      setMotorPwm(MOTOR_RIGHT, controlStep(MOTOR_RIGHT));
    }

    if (logging) {
      Serial.printf("%lu,%.3f,%.1f,%.2f,%.2f,%d,%.2f,%.2f,%d\n",
                    now, kp, setpointRpm,
                    wheel[MOTOR_LEFT].rpmFilt, lastError[MOTOR_LEFT],
                    lastOutput[MOTOR_LEFT],
                    wheel[MOTOR_RIGHT].rpmFilt, lastError[MOTOR_RIGHT],
                    lastOutput[MOTOR_RIGHT]);
    }
  }

  if (!logging && (int32_t)(now - nextDisplay) >= 0) {
    nextDisplay = (nextDisplay == 0 ? now : nextDisplay) + DISPLAY_INTERVAL_MS;
    display();
  }
}
