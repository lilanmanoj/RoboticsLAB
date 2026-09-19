/*
 * Day3Task2.ino
 * Task 2 - Convert Robot Velocity to Wheel Velocities  (lab PDF p.33)
 *
 *     vR = v + (L/2) w
 *     vL = v - (L/2) w
 *
 * Takes the v and w from Task 1, applies the differential-drive inverse
 * kinematics, checks the wheel velocities against the permitted limit, and
 * applies saturation if required.
 *
 * Pure computation - no motors, no encoders.
 *
 * Board: ESP32-S3-DevKitC-1 (runs on any Arduino target)
 */

#include <Arduino.h>

// ASSUMPTION: 100 mm between wheel contact patches. Not recorded anywhere in
// the repository - measure yours centre-to-centre and correct it. Every
// turning calculation depends on it.
float wheelBaseL = 0.100f;   // m

// Permitted wheel speed. Activity 1 reached about 400 RPM, ~0.90 m/s on a
// 43 mm wheel; this leaves margin for the controllers to correct.
float vWheelMax = 0.60f;     // m/s

// Defaults are the Task 1 answer for the lab-sheet numbers.
float v = 0.2500f;           // m/s
float w = 1.1760f;           // rad/s

char cmdBuf[48];
uint8_t cmdLen = 0;

/*
 * Saturation by proportional scaling, not clipping.
 *
 * If vR and vL were clipped independently, their difference would change, and
 * w = (vR - vL)/L would come out different from what was commanded - the robot
 * would turn by the wrong amount. Scaling both by the same factor keeps the
 * ratio, so the path is preserved and only the speed along it drops.
 */
void solve() {
  const float half = wheelBaseL / 2.0f;
  const float vRraw = v + half * w;
  const float vLraw = v - half * w;

  const float peak = max(fabsf(vLraw), fabsf(vRraw));
  const bool saturated = peak > vWheelMax;
  const float scale = (saturated && peak > 0.0f) ? (vWheelMax / peak) : 1.0f;

  const float vL = vLraw * scale;
  const float vR = vRraw * scale;

  // What the scaled commands actually correspond to (forward kinematics).
  const float vEff = (vR + vL) / 2.0f;
  const float wEff = (vR - vL) / wheelBaseL;

  Serial.println();
  Serial.println("--------------------------------------------------------");
  Serial.printf("robot command  v = %.4f m/s   w = %.4f rad/s\n", v, w);
  Serial.printf("wheel base     L = %.4f m   (half = %.4f m)\n",
                wheelBaseL, half);
  Serial.printf("wheel limit    %.4f m/s\n", vWheelMax);
  Serial.println("--------------------------------------------------------");
  Serial.printf("vR = v + (L/2)w   = %.4f + %.4f*%.4f = %.4f m/s\n",
                v, half, w, vRraw);
  Serial.printf("vL = v - (L/2)w   = %.4f - %.4f*%.4f = %.4f m/s\n",
                v, half, w, vLraw);
  Serial.println("--------------------------------------------------------");
  if (saturated) {
    Serial.printf("LIMIT EXCEEDED: peak |v_wheel| = %.4f > %.4f m/s\n",
                  peak, vWheelMax);
    Serial.printf("scaling both wheels by %.4f to preserve the turn ratio\n",
                  scale);
    Serial.printf("  vL %.4f -> %.4f m/s\n", vLraw, vL);
    Serial.printf("  vR %.4f -> %.4f m/s\n", vRraw, vR);
    Serial.printf("effective v = %.4f m/s (was %.4f), w = %.4f rad/s (unchanged)\n",
                  vEff, v, wEff);
  } else {
    Serial.println("within limits - no saturation required");
  }
  Serial.println("--------------------------------------------------------");
  Serial.printf("ANSWER: vL = %.4f m/s, vR = %.4f m/s\n", vL, vR);
  Serial.println();
  Serial.println("Feed vL and vR into Task 3 to get the RPM setpoints.");
}

void printHelp() {
  Serial.println();
  Serial.println("commands (end with Enter):");
  Serial.println("  c<v>,<w>   set robot velocity, e.g. c0.25,1.176");
  Serial.println("  L<m>       set wheel base,     e.g. L0.10");
  Serial.println("  m<m/s>     set wheel limit,    e.g. m0.60");
  Serial.println("  r          recompute and print");
  Serial.println("  h          this help");
  Serial.println();
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
  float vals[2];
  switch (cmd[0]) {
    case 'c':
      if (parseFloats(cmd + 1, vals, 2) == 2) {
        v = vals[0];
        w = vals[1];
        solve();
      } else {
        Serial.println("need two values: c<v>,<w>");
      }
      break;
    case 'L':
      wheelBaseL = max(0.001f, (float)atof(cmd + 1));
      solve();
      break;
    case 'm':
      vWheelMax = max(0.001f, (float)atof(cmd + 1));
      solve();
      break;
    case 'r': solve(); break;
    default: printHelp(); break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== Task 2 - Convert Robot Velocity to Wheel Velocities ===");
  Serial.println("Worked with the Task 1 answer as input:");
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
