/*
 * Day3Activity6.ino
 * Activity 6 - Independent Left and Right Wheel Control  (lab PDF p.27)
 *
 * Runs a separate PI velocity controller per wheel, each with its own gains,
 * setpoint and integrator, so the two wheels can be commanded to different
 * speeds and verified independently against their own setpoints.
 *
 * Because the wheels are then decoupled, this is also the first sketch where
 * the robot does something recognisable: equal setpoints drive straight,
 * unequal ones arc, and equal-and-opposite ones spin in place.
 *
 * Board: ESP32-S3-DevKitC-1
 */

#include <Arduino.h>
#include "RobotBase.h"
#include "VelocityPI.h"

const uint32_t CONTROL_INTERVAL_MS = 20;   // 50 Hz
const uint32_t DISPLAY_INTERVAL_MS = 250;
const float FILTER_ALPHA = 0.30f;

// Separate gains per wheel: Activity 1 measured a 3.3% gain mismatch, and the
// left wheel has a much larger dead-band, so there is no reason to assume one
// tuning suits both.
VelocityPI ctrl[2];
float kp[2] = {0.35f, 0.35f};
float ki[2] = {2.50f, 2.50f};

float setpoint[2] = {0.0f, 0.0f};
bool logging = false;

char cmdBuf[32];
uint8_t cmdLen = 0;

void printHelp() {
  Serial.println();
  Serial.println("commands (end with Enter):");
  Serial.println("  l<rpm>   LEFT wheel setpoint,  e.g. l150");
  Serial.println("  r<rpm>   RIGHT wheel setpoint, e.g. r100");
  Serial.println("  v<rpm>   both wheels the same (drive straight)");
  Serial.println("  t<rpm>   spin in place: left +rpm, right -rpm");
  Serial.println("  a<Kp>    Kp for BOTH wheels, e.g. a0.5");
  Serial.println("  b<Ki>    Ki for BOTH wheels, e.g. b2.5");
  Serial.println("  A<Kp>/B<Ki>  same but LEFT wheel only");
  Serial.println("  C<Kp>/D<Ki>  same but RIGHT wheel only");
  Serial.println("  s        stop");
  Serial.println("  g        toggle CSV logging");
  Serial.println("  h        this help");
  Serial.println();
}

void printCsvHeader() {
  Serial.println("t_ms,set_l,rpm_l,err_l,pwm_l,set_r,rpm_r,err_r,pwm_r");
}

void applyGains() {
  ctrl[MOTOR_LEFT].kp = kp[MOTOR_LEFT];
  ctrl[MOTOR_LEFT].ki = ki[MOTOR_LEFT];
  ctrl[MOTOR_RIGHT].kp = kp[MOTOR_RIGHT];
  ctrl[MOTOR_RIGHT].ki = ki[MOTOR_RIGHT];
}

void handleCommand(const char *cmd) {
  const float arg = atof(cmd + 1);
  switch (cmd[0]) {
    case 'l': setpoint[MOTOR_LEFT] = constrain(arg, -RPM_MAX, RPM_MAX); break;
    case 'r': setpoint[MOTOR_RIGHT] = constrain(arg, -RPM_MAX, RPM_MAX); break;
    case 'v':
      setpoint[MOTOR_LEFT] = constrain(arg, -RPM_MAX, RPM_MAX);
      setpoint[MOTOR_RIGHT] = setpoint[MOTOR_LEFT];
      break;
    case 't':
      setpoint[MOTOR_LEFT] = constrain(arg, -RPM_MAX, RPM_MAX);
      setpoint[MOTOR_RIGHT] = -setpoint[MOTOR_LEFT];
      break;
    case 'a': kp[0] = kp[1] = max(0.0f, arg); applyGains(); break;
    case 'b': ki[0] = ki[1] = max(0.0f, arg); applyGains(); break;
    case 'A': kp[MOTOR_LEFT] = max(0.0f, arg); applyGains(); break;
    case 'B': ki[MOTOR_LEFT] = max(0.0f, arg); applyGains(); break;
    case 'C': kp[MOTOR_RIGHT] = max(0.0f, arg); applyGains(); break;
    case 'D': ki[MOTOR_RIGHT] = max(0.0f, arg); applyGains(); break;
    case 's':
      setpoint[0] = setpoint[1] = 0.0f;
      piReset(ctrl[0]);
      piReset(ctrl[1]);
      motorsCoast();
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

void displayRow(const char *name, MotorId m) {
  Serial.printf(" %-5s |%9.1f%9.1f%9.1f |%7.2f%7.2f |%6d\033[K\n",
                name, setpoint[m], wheel[m].rpmFilt, ctrl[m].error,
                kp[m], ki[m], ctrl[m].output);
}

void display() {
  Serial.printf("\033[4A");
  Serial.printf(" wheel |  set rpm  act rpm  err rpm |     Kp     Ki |   pwm\033[K\n");
  Serial.printf(" ------+---------------------------+---------------+------\033[K\n");
  displayRow("LEFT", MOTOR_LEFT);
  displayRow("RIGHT", MOTOR_RIGHT);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  robotBaseBegin();
  piInit(ctrl[MOTOR_LEFT], kp[MOTOR_LEFT], ki[MOTOR_LEFT]);
  piInit(ctrl[MOTOR_RIGHT], kp[MOTOR_RIGHT], ki[MOTOR_RIGHT]);

  Serial.println();
  Serial.println("=== Activity 6 - Independent Left and Right Wheel Control ===");
  Serial.println("Each wheel has its own PI controller, gains and integrator.");
  Serial.println("Put the robot on the floor for the motion tests.");
  printHelp();
  Serial.println("\n\n\n");
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

    for (int i = 0; i < 2; i++) {
      const MotorId m = (MotorId)i;
      if (setpoint[m] == 0.0f) {
        setMotorPwm(m, 0);
        piReset(ctrl[m]);
        ctrl[m].error = -wheel[m].rpmFilt;
      } else {
        setMotorPwm(m, piUpdate(ctrl[m], m, setpoint[m], wheel[m].rpmFilt, dt));
      }
    }

    if (logging) {
      Serial.printf("%lu,%.1f,%.2f,%.2f,%d,%.1f,%.2f,%.2f,%d\n", now,
                    setpoint[MOTOR_LEFT], wheel[MOTOR_LEFT].rpmFilt,
                    ctrl[MOTOR_LEFT].error, ctrl[MOTOR_LEFT].output,
                    setpoint[MOTOR_RIGHT], wheel[MOTOR_RIGHT].rpmFilt,
                    ctrl[MOTOR_RIGHT].error, ctrl[MOTOR_RIGHT].output);
    }
  }

  if (!logging && (int32_t)(now - nextDisplay) >= 0) {
    nextDisplay = (nextDisplay == 0 ? now : nextDisplay) + DISPLAY_INTERVAL_MS;
    display();
  }
}
