# Activity 6 — Independent Left and Right Wheel Control

Lab PDF p.27. A separate PI velocity controller per wheel, each with its own
gains, setpoint and integrator, so the wheels can be commanded independently
and verified against their own setpoints.

This is the first sketch where the robot does something recognisable: equal
setpoints drive straight, unequal ones arc, equal-and-opposite spin in place.

## Build and run

```bash
FQBN="esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled,CDCOnBoot=cdc"

arduino-cli compile --fqbn "$FQBN" Day3Activity6
arduino-cli upload -p /dev/ttyACM0 --fqbn "$FQBN" Day3Activity6
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

Bench-test on blocks first, then put the robot on the floor for the motion
tests. You'll need a USB cable long enough, or see the power notes in the
repository [README.md](../README.md).

## Commands

| Command | Effect |
| --- | --- |
| `l150` | LEFT wheel setpoint |
| `r100` | RIGHT wheel setpoint |
| `v150` | both wheels the same — drive straight |
| `t150` | spin in place: left `+150`, right `−150` |
| `a0.5` / `b2.5` | `Kp` / `Ki` for **both** wheels |
| `A0.5` / `B2.5` | `Kp` / `Ki` for the **left** wheel only |
| `C0.5` / `D2.5` | `Kp` / `Ki` for the **right** wheel only |
| `s` | stop |
| `g` | toggle CSV logging |
| `h` | help |

## Live display

```
 wheel |  set rpm  act rpm  err rpm |     Kp     Ki |   pwm
 ------+---------------------------+---------------+------
 LEFT  |    150.0    149.8      0.2 |   0.35   2.50 |    97
 RIGHT |    100.0    100.1     -0.1 |   0.35   2.50 |    72
```

## The verification the activity asks for

"Verify that the measured wheel speeds follow their individual setpoints."

```
l150
r100
```

Both `err rpm` columns should converge to roughly zero within a second or so,
each at its *own* setpoint. That is the whole point — the two loops are
independent, and the right wheel holding 100 while the left holds 150 proves it.

Then observe the motion:

| Command | Expected motion |
| --- | --- |
| `v150` | straight ahead |
| `l150` then `r100` | arcs to the right (slower right wheel) |
| `t150` | spins in place, counter-clockwise |
| `l150` `r0` | pivots about the stationary right wheel |

## Why per-wheel gains

Activity 1 measured a **3.3% gain mismatch** between the motors and a much
larger dead-band on the left. There is no reason to assume one tuning suits
both, so this sketch gives each wheel its own `Kp` and `Ki`, adjustable
separately with `A`/`B` and `C`/`D`.

In practice the same gains usually work for both — the integrators absorb the
mismatch. The separate controls are there so you can demonstrate that they do,
and tune around the left wheel's stiction if it needs it.

## Logging

Press `g`, then:

```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200 | tee activity6_log.csv
```

Columns:

```
t_ms,set_l,rpm_l,err_l,pwm_l,set_r,rpm_r,err_r,pwm_r
```

## Expected trouble

**Driving straight is not the same as commanding equal RPM.** Even with both
wheels tracking their setpoints perfectly, the robot will drift if the two
wheels differ in diameter, or if one has more slip. Closed-loop velocity
control fixes the *motor* mismatch, not the *wheel* mismatch. Correcting that
needs the heading feedback of Task 4.

**Below ~90 RPM the left wheel may stall** — Activity 1 measured it not
breaking away until PWM ≈ 50. Raise the setpoint or accept that the bottom of
the range is stiction-dominated.

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
