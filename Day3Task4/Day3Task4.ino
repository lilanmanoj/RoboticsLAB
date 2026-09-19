/*
 * Day3Task4.ino
 * Task 4 - Implement the Position-to-Velocity Controller  (lab PDF p.35)
 *
 *   Read encoders -> estimate x,y,theta -> rho,theta_d,e_theta -> v,w
 *                 -> vL,vR -> wheel velocity controllers
 *
 * The whole chain from the lab sheet, executed every control cycle. Tasks 1-3
 * computed one snapshot of this by hand; here the same arithmetic runs at
 * 50 Hz with encoder-based odometry closing the outer position loop and the
 * Activity 4 PI controllers closing the inner velocity loops.
 *
 * Board: ESP32-S3-DevKitC-1
 */

#include <Arduino.h>
#include "RobotBase.h"
#include "VelocityPI.h"

const uint32_t CONTROL_INTERVAL_MS = 20;   // 50 Hz
const uint32_t DISPLAY_INTERVAL_MS = 250;
const float FILTER_ALPHA = 0.30f;

// Outer-loop gains and limits, from the Task 1 lab sheet.
float kv = 0.5f;          // 1/s
float kw = 2.0f;          // 1/s
float vMax = 0.25f;       // m/s
float wMax = 1.5f;        // rad/s
const float V_WHEEL_MAX = 0.60f;

// Stop when this close to the target. Below roughly one wheel radius the
// heading error becomes meaningless - atan2 of two tiny numbers is noise -
// so the robot would spin on the spot chasing it.
float arriveTolM = 0.05f;

// Inner-loop gains.
float kp = 0.35f;
float ki = 2.50f;
VelocityPI ctrl[2];

// Odometry state (PDF p.30-31).
float poseX = 0.0f, poseY = 0.0f, poseTheta = 0.0f;
float targetX = 1.5f, targetY = 1.0f;

// Latest outer-loop quantities, kept for display.
float rho = 0.0f, thetaD = 0.0f, eTheta = 0.0f;
float cmdV = 0.0f, cmdW = 0.0f;
float wheelSetRpm[2] = {0.0f, 0.0f};
bool running = false;
bool arrived = false;
bool logging = false;

char cmdBuf[48];
uint8_t cmdLen = 0;

inline float rad2deg(float r) { return r * 180.0f / PI; }
inline float deg2rad(float d) { return d * PI / 180.0f; }

float wrapPi(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a <= -PI) a += 2.0f * PI;
  return a;
}

/*
 * Odometry from the encoder counts of this cycle.
 *
 * Heading is advanced by half the increment before projecting the distance,
 * which places the displacement along the chord of the arc rather than along
 * the heading at the start of it. On a straight run it makes no difference; on
 * a tight turn the plain version accumulates a consistent outward bias.
 */
void updateOdometry() {
  const float perCount = PI * WHEEL_DIAMETER_M / COUNTS_PER_OUTPUT_REV;
  const float dsL = wheel[MOTOR_LEFT].deltaCounts * perCount;
  const float dsR = wheel[MOTOR_RIGHT].deltaCounts * perCount;

  const float ds = (dsR + dsL) / 2.0f;
  const float dTheta = (dsR - dsL) / WHEEL_BASE_M;

  poseX += ds * cosf(poseTheta + dTheta / 2.0f);
  poseY += ds * sinf(poseTheta + dTheta / 2.0f);
  poseTheta = wrapPi(poseTheta + dTheta);
}

// Task 1 arithmetic, run every cycle against the live pose estimate.
void outerLoop() {
  const float dx = targetX - poseX;
  const float dy = targetY - poseY;

  rho = sqrtf(dx * dx + dy * dy);
  thetaD = atan2f(dy, dx);
  eTheta = wrapPi(thetaD - poseTheta);

  if (rho < arriveTolM) {
    arrived = true;
    running = false;
    cmdV = 0.0f;
    cmdW = 0.0f;
    return;
  }

  cmdV = constrain(kv * rho, -vMax, vMax);
  cmdW = constrain(kw * eTheta, -wMax, wMax);

  // Do not drive forward while badly mis-aimed: turn on the spot first, or the
  // robot loops around the target instead of converging on it.
  if (fabsf(eTheta) > deg2rad(45.0f)) {
    cmdV = 0.0f;
  }
}

// Task 2 + Task 3: robot velocity -> wheel velocities -> RPM setpoints.
void innerSetpoints() {
  const float half = WHEEL_BASE_M / 2.0f;
  float vR = cmdV + half * cmdW;
  float vL = cmdV - half * cmdW;

  const float peak = max(fabsf(vL), fabsf(vR));
  if (peak > V_WHEEL_MAX && peak > 0.0f) {
    const float scale = V_WHEEL_MAX / peak;   // scale both, preserve the turn
    vL *= scale;
    vR *= scale;
  }

  wheelSetRpm[MOTOR_LEFT] = mpsToRpm(vL);
  wheelSetRpm[MOTOR_RIGHT] = mpsToRpm(vR);
}

void printHelp() {
  Serial.println();
  Serial.println("commands (end with Enter):");
  Serial.println("  t<xd>,<yd>  set target,   e.g. t1.5,1.0");
  Serial.println("  p<x>,<y>,<deg>  set/reset pose, e.g. p0,0,0");
  Serial.println("  o           reset odometry to (0,0,0)");
  Serial.println("  G           GO - start driving to the target");
  Serial.println("  s          stop");
  Serial.println("  a<m>       arrival tolerance, e.g. a0.05");
  Serial.println("  v<m/s>     v_max      w<rad/s> w_max");
  Serial.println("  k<Kv>      outer linear gain   j<Kw> outer angular gain");
  Serial.println("  P<Kp>      inner Kp            I<Ki>  inner Ki");
  Serial.println("  g          toggle CSV logging");
  Serial.println("  h          this help");
  Serial.println();
}

void printCsvHeader() {
  Serial.println("t_ms,x,y,theta_deg,rho,theta_d_deg,e_theta_deg,cmd_v,cmd_w,"
                 "set_rpm_l,set_rpm_r,rpm_l,rpm_r,pwm_l,pwm_r");
}

void resetOdometry(float x, float y, float thetaDeg) {
  poseX = x;
  poseY = y;
  poseTheta = deg2rad(thetaDeg);
  arrived = false;
}

int parseFloats(const char *s, float *out, int maxN) {
  int n = 0;
  const char *p = s;
  while (n < maxN && *p) {
    out[n++] = atof(p);
    const char *comma = strchr(p, ',');
    if (!comma) break;
    p = comma + 1;
  }
  return n;
}

void handleCommand(const char *cmd) {
  float vals[3];
  const float arg = atof(cmd + 1);
  switch (cmd[0]) {
    case 't':
      if (parseFloats(cmd + 1, vals, 2) == 2) {
        targetX = vals[0];
        targetY = vals[1];
        arrived = false;
      }
      break;
    case 'p':
      if (parseFloats(cmd + 1, vals, 3) == 3) {
        resetOdometry(vals[0], vals[1], vals[2]);
      }
      break;
    case 'o': resetOdometry(0.0f, 0.0f, 0.0f); break;
    case 'G':
      arrived = false;
      running = true;
      piReset(ctrl[0]);
      piReset(ctrl[1]);
      break;
    case 's':
      running = false;
      cmdV = cmdW = 0.0f;
      wheelSetRpm[0] = wheelSetRpm[1] = 0.0f;
      motorsCoast();
      break;
    case 'a': arriveTolM = max(0.005f, arg); break;
    case 'v': vMax = max(0.0f, arg); break;
    case 'w': wMax = max(0.0f, arg); break;
    case 'k': kv = max(0.0f, arg); break;
    case 'j': kw = max(0.0f, arg); break;
    case 'P': kp = max(0.0f, arg); ctrl[0].kp = ctrl[1].kp = kp; break;
    case 'I': ki = max(0.0f, arg); ctrl[0].ki = ctrl[1].ki = ki; break;
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
  Serial.printf("\033[7A");
  Serial.printf(" state    %-8s   target (%.3f, %.3f)\033[K\n",
                arrived ? "ARRIVED" : (running ? "RUNNING" : "idle"),
                targetX, targetY);
  Serial.printf(" pose     x %7.3f m   y %7.3f m   theta %7.2f deg\033[K\n",
                poseX, poseY, rad2deg(poseTheta));
  Serial.printf(" errors   rho %6.3f m  theta_d %7.2f deg  e_theta %7.2f deg\033[K\n",
                rho, rad2deg(thetaD), rad2deg(eTheta));
  Serial.printf(" command  v %6.3f m/s   w %6.3f rad/s\033[K\n", cmdV, cmdW);
  Serial.printf(" wheel |  set rpm  act rpm  err rpm |   pwm\033[K\n");
  Serial.printf(" LEFT  |%9.1f%9.1f%9.1f |%6d\033[K\n",
                wheelSetRpm[MOTOR_LEFT], wheel[MOTOR_LEFT].rpmFilt,
                ctrl[MOTOR_LEFT].error, ctrl[MOTOR_LEFT].output);
  Serial.printf(" RIGHT |%9.1f%9.1f%9.1f |%6d\033[K\n",
                wheelSetRpm[MOTOR_RIGHT], wheel[MOTOR_RIGHT].rpmFilt,
                ctrl[MOTOR_RIGHT].error, ctrl[MOTOR_RIGHT].output);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  robotBaseBegin();
  piInit(ctrl[MOTOR_LEFT], kp, ki);
  piInit(ctrl[MOTOR_RIGHT], kp, ki);

  Serial.println();
  Serial.println("=== Task 4 - Position-to-Velocity Controller ===");
  Serial.println("read encoders -> x,y,theta -> rho,theta_d,e_theta -> v,w");
  Serial.println("              -> vL,vR -> wheel PI controllers");
  Serial.printf("L = %.3f m, r = %.4f m, Kv = %.2f, Kw = %.2f\n",
                WHEEL_BASE_M, WHEEL_RADIUS_M, kv, kw);
  Serial.println("Needs floor space. Send 'G' to go, 's' to stop.");
  printHelp();
  Serial.println("\n\n\n\n\n\n");
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
    updateOdometry();   // odometry runs always, so the pose stays live

    if (running) {
      outerLoop();
      innerSetpoints();
    } else {
      wheelSetRpm[0] = wheelSetRpm[1] = 0.0f;
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
      Serial.printf("%lu,%.4f,%.4f,%.2f,%.4f,%.2f,%.2f,%.4f,%.4f,"
                    "%.1f,%.1f,%.2f,%.2f,%d,%d\n",
                    now, poseX, poseY, rad2deg(poseTheta), rho,
                    rad2deg(thetaD), rad2deg(eTheta), cmdV, cmdW,
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
