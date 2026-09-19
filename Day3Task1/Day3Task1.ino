/*
 * Day3Task1.ino
 * Task 1 - Calculate Motion Toward a Target  (lab PDF p.32)
 *
 *     rho    = sqrt((xd - x)^2 + (yd - y)^2)
 *     theta_d = atan2(yd - y, xd - x)
 *     e_theta = theta_d - theta
 *     v = Kv * rho          (saturated at v_max)
 *     w = Kw * e_theta      (saturated at w_max)
 *
 * Pure computation - no motors, no encoders. Prints the worked answer for the
 * values given in the lab sheet at boot, then recomputes for any pose and
 * target you type in, so the same code doubles as a checker for your own
 * hand calculations.
 *
 * Board: ESP32-S3-DevKitC-1 (runs on any Arduino target)
 */

#include <Arduino.h>

// --- Given in the lab sheet (PDF p.32) -------------------------------------
const float KV = 0.5f;          // 1/s
const float KW = 2.0f;          // 1/s
const float V_MAX = 0.25f;      // m/s
const float W_MAX = 1.5f;       // rad/s

float x = 0.0f, y = 0.0f, theta = 0.0f;   // robot pose, theta in radians
float xd = 1.5f, yd = 1.0f;               // target
// ---------------------------------------------------------------------------

char cmdBuf[48];
uint8_t cmdLen = 0;

inline float rad2deg(float r) { return r * 180.0f / PI; }
inline float deg2rad(float d) { return d * PI / 180.0f; }

/*
 * Wrap an angle to (-pi, pi].
 *
 * Without this, a heading error of +350 degrees would be acted on as a long
 * way round to the left instead of a short hop to the right. Every heading
 * error in a real controller must be wrapped.
 */
float wrapPi(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a <= -PI) a += 2.0f * PI;
  return a;
}

void solve() {
  const float dx = xd - x;
  const float dy = yd - y;

  const float rho = sqrtf(dx * dx + dy * dy);
  const float thetaD = atan2f(dy, dx);
  const float eTheta = wrapPi(thetaD - theta);

  const float vRaw = KV * rho;
  const float wRaw = KW * eTheta;
  const float v = constrain(vRaw, -V_MAX, V_MAX);
  const float w = constrain(wRaw, -W_MAX, W_MAX);

  Serial.println();
  Serial.println("--------------------------------------------------------");
  Serial.printf("robot pose     x = %.4f m   y = %.4f m   theta = %.4f rad (%.2f deg)\n",
                x, y, theta, rad2deg(theta));
  Serial.printf("target         xd = %.4f m  yd = %.4f m\n", xd, yd);
  Serial.printf("gains          Kv = %.2f 1/s   Kw = %.2f 1/s\n", KV, KW);
  Serial.printf("limits         v_max = %.2f m/s  w_max = %.2f rad/s\n",
                V_MAX, W_MAX);
  Serial.println("--------------------------------------------------------");
  Serial.printf("dx, dy         %.4f m, %.4f m\n", dx, dy);
  Serial.printf("rho            sqrt(%.4f^2 + %.4f^2)      = %.4f m\n",
                dx, dy, rho);
  Serial.printf("theta_d        atan2(%.4f, %.4f)          = %.4f rad (%.2f deg)\n",
                dy, dx, thetaD, rad2deg(thetaD));
  Serial.printf("e_theta        %.4f - %.4f                = %.4f rad (%.2f deg)\n",
                thetaD, theta, eTheta, rad2deg(eTheta));
  Serial.println("--------------------------------------------------------");
  Serial.printf("v = Kv*rho     %.2f * %.4f = %.4f m/s   -> %.4f m/s%s\n",
                KV, rho, vRaw, v, fabsf(vRaw) > V_MAX ? "  [SATURATED]" : "");
  Serial.printf("w = Kw*e_th    %.2f * %.4f = %.4f rad/s -> %.4f rad/s%s\n",
                KW, eTheta, wRaw, w, fabsf(wRaw) > W_MAX ? "  [SATURATED]" : "");
  Serial.println("--------------------------------------------------------");
  Serial.printf("ANSWER: rho = %.4f m, theta_d = %.2f deg, e_theta = %.2f deg,\n",
                rho, rad2deg(thetaD), rad2deg(eTheta));
  Serial.printf("        v = %.4f m/s, w = %.4f rad/s\n", v, w);
  Serial.println();
  Serial.println("Feed v and w into Task 2 to get the wheel velocities.");
}

void printHelp() {
  Serial.println();
  Serial.println("commands (end with Enter):");
  Serial.println("  p<x>,<y>,<theta_deg>   set robot pose,  e.g. p0,0,0");
  Serial.println("  t<xd>,<yd>             set target,      e.g. t1.5,1.0");
  Serial.println("  r                      recompute and print");
  Serial.println("  h                      this help");
  Serial.println();
}

// Parses up to three comma-separated floats after the command letter.
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
  switch (cmd[0]) {
    case 'p':
      if (parseFloats(cmd + 1, vals, 3) == 3) {
        x = vals[0];
        y = vals[1];
        theta = deg2rad(vals[2]);
        solve();
      } else {
        Serial.println("need three values: p<x>,<y>,<theta_deg>");
      }
      break;
    case 't':
      if (parseFloats(cmd + 1, vals, 2) == 2) {
        xd = vals[0];
        yd = vals[1];
        solve();
      } else {
        Serial.println("need two values: t<xd>,<yd>");
      }
      break;
    case 'r': solve(); break;
    default: printHelp(); break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== Task 1 - Calculate Motion Toward a Target ===");
  Serial.println("Worked answer for the values given in the lab sheet:");
  solve();
  printHelp();
}

void loop() {
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
