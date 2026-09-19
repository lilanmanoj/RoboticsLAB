# Activity 7 — Command Robot Velocity

Lab PDF p.28. Commands the robot as a whole — linear velocity `v` and angular
velocity `ω` — and lets the differential-drive kinematics decide what each
wheel does.

```
vR = v + (L/2)ω
vL = v − (L/2)ω
ω_wheel = v_wheel / r
RPM = ω_wheel × 60 / (2π)
```

The resulting RPM values become setpoints for the Activity 6 wheel controllers.
Encoder feedback is then run back through the **forward** kinematics to report
the `v` and `ω` actually achieved — the verification step the activity asks for.

## Build and run

```bash
FQBN="esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled,CDCOnBoot=cdc"

arduino-cli compile --fqbn "$FQBN" Day3Activity7
arduino-cli upload -p /dev/ttyACM0 --fqbn "$FQBN" Day3Activity7
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

**Measure `WHEEL_BASE_M` and `WHEEL_DIAMETER_M` before trusting anything here.**
Both are assumptions in `RobotBase.h`, and every number on screen depends on
them. See the table at the bottom.

## Commands

| Command | Effect |
| --- | --- |
| `v0.20` | desired linear velocity, m/s |
| `w0.8` | desired angular velocity, rad/s (positive turns left) |
| `c0.20,0.8` | both at once |
| `k0.5` / `i2.5` | inner-loop `Kp` / `Ki` |
| `s` | stop |
| `g` | toggle CSV logging |
| `h` | help |

## Live display

```
 commanded  v  0.200 m/s   w  0.800 rad/s
 measured   v  0.198 m/s   w  0.791 rad/s
 wheel |  set m/s  set rpm  act rpm  err rpm |   pwm
 ------+-----------------------------------+------
 LEFT  |    0.160    71.1     71.0      0.1 |    52
 RIGHT |    0.240   106.6    106.4      0.2 |    76
```

The top two lines are the verification: **commanded** goes through inverse
kinematics to the wheels, **measured** comes back from the encoders through
forward kinematics. They should agree to within a percent or two.

## Suggested test sequence

| Command | Expected |
| --- | --- |
| `c0.2,0` | straight ahead, both wheels equal |
| `c0,1.0` | spin in place, wheels equal and opposite |
| `c0.2,0.8` | arc to the left, right wheel faster |
| `c0.2,-0.8` | arc to the right |
| `c0.5,2.0` | triggers saturation — see below |

## Saturation, and why it scales rather than clips

When a demand exceeds what the wheels can deliver, `computeWheelSetpoints()`
scales **both** wheel speeds by the same factor instead of clipping each one:

```c
const float scale = V_WHEEL_MAX / peak;
vL *= scale;
vR *= scale;
```

Clipping independently would change the *difference* between the wheels, and
since `ω = (vR − vL)/L`, the robot would turn by a different amount than
commanded — you'd ask for a gentle arc and get a sharp one. Scaling preserves
the ratio, so the path stays the same shape and only the speed along it drops.

The display shows `[SATURATED - scaled]` when this is active. Try `c0.5,2.0`
and watch `measured v` come out well below `commanded v` while `measured w`
stays close.

`V_WHEEL_MAX` is set to 0.60 m/s. Activity 1 reached about 400 RPM (~0.90 m/s
on a 43 mm wheel), so this leaves the controllers headroom to correct.

## Logging

Press `g`, then capture:

```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200 | tee activity7_log.csv
```

Columns:

```
t_ms,cmd_v,cmd_w,set_mps_l,set_mps_r,set_rpm_l,set_rpm_r,
rpm_l,rpm_r,mps_l,mps_r,meas_v,meas_w,pwm_l,pwm_r
```

Plotting `cmd_v` against `meas_v` and `cmd_w` against `meas_w` is the clearest
figure for this activity.

## Expected trouble

**`measured ω` will be the less accurate of the two.** It comes from the
*difference* of two wheel speeds divided by `L`, so it inherits both wheels'
measurement noise and is directly proportional to your `WHEEL_BASE_M`
assumption. A 10% error in `L` is a 10% error in every ω you report.

**`measured v` and `measured ω` are what the wheels did, not what the robot
did.** Wheel slip is invisible to an encoder. On a slippery floor the encoders
will happily report a perfect turn the robot never made.

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
