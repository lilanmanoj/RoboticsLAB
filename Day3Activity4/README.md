# Activity 4 — PI Velocity Control

Lab PDF p.25.

```
u(t) = Kp e(t) + Ki ∫ e(t) dt
```

Adds the integral term Activity 3 lacked, so the steady-state velocity error is
driven to zero rather than merely made small.

## Build and run

```bash
FQBN="esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled,CDCOnBoot=cdc"

arduino-cli compile --fqbn "$FQBN" Day3Activity4
arduino-cli upload -p /dev/ttyACM0 --fqbn "$FQBN" Day3Activity4
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

Lift the chassis so the wheels spin free.

## Commands

| Command | Effect |
| --- | --- |
| `v150` | desired wheel velocity |
| `k0.5` | set `Kp` |
| `i2.5` | set `Ki` — **`i0` gives pure P**, for the comparison |
| `w` | toggle feedforward |
| `z` | zero the integrators |
| `s` | stop |
| `g` | toggle CSV logging |
| `h` | help |

## Live display

```
 Kp 0.350  Ki 2.500  ff ON   setpoint  150.0 rpm
 wheel |  set rpm  act rpm  err rpm   integral |   pwm
 LEFT  |    150.0    149.8      0.2       1.43 |    97
 RIGHT |    150.0    150.1     -0.1       2.07 |   101
```

The `integral` column is the accumulated error in RPM·seconds. Watching it is
the fastest way to understand what the I term is doing: it climbs while the
error is positive and holds a constant value once the error reaches zero — that
held value *is* the standing PWM the P term could never supply.

## The comparison the activity asks for

"Compare P and PI controller performance by recording the response for the same
desired velocity."

```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200 | tee activity4_compare.csv
```

Then:

```
g            <- start logging
i0           <- pure P
v150         <- let it settle ~5 s, note the standing error
s
i2.5         <- PI
v150         <- let it settle ~5 s, error should reach 0
s
```

Both runs land in one CSV with the `ki` column marking which is which. Columns:

```
t_ms,kp,ki,setpoint_rpm,rpm_l,err_l,int_l,pwm_l,rpm_r,err_r,int_r,pwm_r
```

## Tuning suggestion

Start `Kp = 0.35`, `Ki = 2.5` (the defaults) and adjust from there:

| Symptom | Try |
| --- | --- |
| slow to reach setpoint | raise `Kp` |
| error never quite reaches zero | raise `Ki` |
| oscillation at constant speed | lower `Kp` |
| overshoot then slow crawl back | lower `Ki` |
| sluggish after a big step | integral windup — see below |

Change one gain at a time and let it settle for several seconds before judging.

## Integral windup — the one thing that will bite you

If you command a speed the motor cannot reach, the error stays positive
forever and `∫e dt` grows without bound. When you then lower the setpoint, the
controller ignores you until that accumulated integral unwinds — the wheel
keeps running flat out for seconds.

`controlStep()` guards against it by freezing the integrator whenever the output
is already at a rail and the error would push it further in:

```c
const bool saturated = (candidate > 255.0f && e > 0.0f) ||
                       (candidate < -255.0f && e < 0.0f);
if (!saturated) {
  integral[m] += e * dt;
}
```

To see the failure mode deliberately, comment that condition out and command
`v600` (unreachable), then `v100`. Press `z` to recover.

## Why feedforward is on by default here

Unlike Activity 3, feedforward starts **on**. The Activity 1 model supplies the
bulk of the PWM, so the integrator only has to absorb the residual — it starts
near zero instead of having to build the whole operating point, which makes
tuning much less touchy. Press `w` to turn it off and see how much more work
the integrator has to do.

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
