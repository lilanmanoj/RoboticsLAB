# Task 4 — Implement the Position-to-Velocity Controller

Lab PDF p.35.

```
Read encoders → estimate x,y,θ → calculate ρ,θd,eθ → calculate v,ω
              → calculate vL,vR → wheel velocity controllers
```

The whole chain, executed every control cycle. Tasks 1–3 computed one snapshot
of this by hand; here the same arithmetic runs at 50 Hz with encoder odometry
closing the outer position loop and the Activity 4 PI controllers closing the
inner velocity loops.

## Build and run

```bash
FQBN="esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled,CDCOnBoot=cdc"

arduino-cli compile --fqbn "$FQBN" Day3Task4
arduino-cli upload -p /dev/ttyACM0 --fqbn "$FQBN" Day3Task4
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

**This one drives the robot across the floor.** Clear about 2 × 1.5 m, place
the robot at the origin facing +x, and keep `s` (stop) within reach.

Start on blocks with the wheels free to confirm the setpoints look sane before
letting it loose.

## Commands

| Command | Effect |
| --- | --- |
| `t1.5,1.0` | set target `xd, yd` in metres |
| `p0,0,0` | set pose to `x, y, θ°` |
| `o` | reset odometry to `(0, 0, 0)` |
| `G` | **GO** — start driving to the target |
| `s` | stop |
| `a0.05` | arrival tolerance, metres |
| `v0.25` / `w1.5` | `v_max` / `ω_max` |
| `k0.5` / `j2.0` | outer gains `Kv` / `Kω` |
| `P0.35` / `I2.5` | inner wheel-loop `Kp` / `Ki` |
| `g` | toggle CSV logging |
| `h` | help |

## Typical session

```
o                <- zero the odometry where the robot stands
t1.5,1.0         <- lab-sheet target
G                <- go
```

Watch `rho` count down. The robot turns to face the target first, then drives.
It stops and reports `ARRIVED` when `rho < 0.05 m`.

## Live display

```
 state    RUNNING    target (1.500, 1.000)
 pose     x   0.412 m   y   0.271 m   theta   33.42 deg
 errors   rho  1.307 m  theta_d   33.69 deg  e_theta    0.27 deg
 command  v  0.250 m/s   w  0.009 rad/s
 wheel |  set rpm  act rpm  err rpm |   pwm
 LEFT  |    110.9    110.7      0.2 |    76
 RIGHT |    111.3    111.1      0.2 |    78
```

## Three design decisions worth knowing

**Odometry advances heading by half the increment before projecting:**

```c
poseX += ds * cosf(poseTheta + dTheta / 2.0f);
```

This places the displacement along the chord of the arc rather than along the
heading at the start of it. On a straight run it changes nothing; on a tight
turn the naive version accumulates a consistent outward bias.

**Forward motion is suppressed while badly mis-aimed:**

```c
if (fabsf(eTheta) > deg2rad(45.0f)) {
  cmdV = 0.0f;
}
```

Turn on the spot first, then drive. Without this the robot spirals around the
target instead of converging on it, because it keeps driving forward while
still rotating.

**Arrival tolerance exists because `atan2` becomes noise near the target.**
Within a wheel radius or so, `θd` is the arctangent of two tiny numbers
dominated by odometry error, so `eθ` swings wildly and the robot spins on the
spot chasing it. Stopping at 5 cm avoids that. Tighten it with `a` and you will
see the spin appear.

## Logging and plotting the path

Press `g`, then:

```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200 | tee task4_run.csv
```

Columns:

```
t_ms,x,y,theta_deg,rho,theta_d_deg,e_theta_deg,cmd_v,cmd_w,
set_rpm_l,set_rpm_r,rpm_l,rpm_r,pwm_l,pwm_r
```

Plotting `y` against `x` gives the robot's believed path — the natural figure
for this task. Plotting `rho` against time shows the convergence.

## Expected trouble — read this before judging the result

**Odometry drifts, and there is no correction for it.** The pose comes purely
from integrating wheel counts. Every source of error accumulates and never
washes out: wheel slip, the unverified `COUNTS_PER_OUTPUT_REV`, the assumed
wheel diameter, the assumed wheel base, and any difference between the two
wheels' actual diameters.

Expect the robot to *believe* it arrived while sitting some distance from the
true target. **Measure the real final position with a tape and report both** —
the gap between believed and actual is the interesting result of this task, not
a failure of the code.

The three constants that dominate that gap:

- `WHEEL_DIAMETER_M` — scales all distances. 10% error → 10% distance error.
- `WHEEL_BASE_M` — scales all rotations. 10% error → 10% heading error, which
  compounds into position error over the run.
- `COUNTS_PER_OUTPUT_REV` — still unverified, and likely ~1.6× off.

Calibrate these before drawing conclusions. A straight 1 m run measured with a
tape calibrates the first; a commanded 360° spin measured against a floor mark
calibrates the second.

## Shared files

`RobotBase.h` — pin map, calibration constants, x4 quadrature decoding, motor
output and wheel velocity measurement. An identical copy lives in every sketch
folder that needs it, so each project compiles standalone. **If you change a
calibration constant, change it in all of them.**

## Calibration constants you must check

| Constant | Value | Status |
| --- | --- | --- |
| `COUNTS_PER_OUTPUT_REV` | 840 | **Unverified.** Activity 1 measured ~420 RPM at full duty on a 5 V rail, but these are 300 RPM/6 V motors, so full duty should give *under* 300 RPM. Likely wrong by ~1.6×. Calibrate: mark a wheel, turn it exactly 10 revolutions by hand, `counts / 10` is the true value. |
| `WHEEL_DIAMETER_M` | 0.043 | **Assumption.** Not recorded anywhere in the repository. Measure yours. Affects every m/s figure; RPM figures are unaffected. |
| `WHEEL_BASE_M` | 0.100 | **Assumption.** Measure centre-to-centre across your chassis. Affects all turning kinematics. |

The pin map in these sketches deliberately differs from the repository
[README.md](../README.md): the motor-to-encoder pairing is crossed on this
build, verified on the bench with [MotorDiag](../MotorDiag/). Channel A turns
the encoder on GPIO 36/37, channel B turns the one on GPIO 38/39.
