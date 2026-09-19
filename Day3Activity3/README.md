# Activity 3 — Proportional Velocity Control

Lab PDF p.24.

```
e(t) = ωd − ω(t)
u(t) = Kp e(t)        u is the PWM command
```

Closes the loop on wheel speed with a pure proportional controller, so the
characteristic weakness of P control is visible: a steady-state error that
shrinks but never vanishes as `Kp` rises, and overshoot once `Kp` gets large.

## Build and run

```bash
FQBN="esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled,CDCOnBoot=cdc"

arduino-cli compile --fqbn "$FQBN" Day3Activity3
arduino-cli upload -p /dev/ttyACM0 --fqbn "$FQBN" Day3Activity3
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

Lift the chassis so the wheels spin free.

## Commands

| Command | Effect |
| --- | --- |
| `v150` | desired wheel velocity, 150 RPM (negative reverses) |
| `k0.8` | set `Kp` |
| `w` | toggle feedforward on/off |
| `s` | stop |
| `g` | toggle CSV logging of every control cycle |
| `h` | help |

## Live display

```
 Kp 0.350  feedforward OFF  setpoint  150.0 rpm
 wheel |  set rpm  act rpm  err rpm |   pwm
 LEFT  |    150.0    121.4     28.6 |    10
 RIGHT |    150.0    118.9     31.1 |    11
```

Repaints in place with ANSI cursor-up. Press `g` for a clean CSV stream when
piping to a file.

## What to do — the actual experiment

The activity asks you to *investigate the effect of increasing Kp*. Run this
sequence and record the steady-state error each time:

```
v150        <- pick a setpoint in the middle of the linear region
k0.2        <- watch the error
k0.4
k0.8
k1.5
k3.0        <- expect audible oscillation and overshoot
```

| Kp | What you should see |
| --- | --- |
| small | large steady-state error, slow, no overshoot |
| medium | error shrinks roughly as `1/(1 + Kp·gain)`, response quickens |
| large | error small but the wheel oscillates and overshoots on each change |

**The steady-state error never reaches zero.** That is not a tuning failure —
it is structural. At steady state the error is the *only* thing generating the
PWM, so if the error were zero the output would be zero and the wheel would
stop. A P controller needs a standing error to hold a standing output. Removing
it is what Activity 4's integral term is for.

## Why there is a feedforward toggle

The PDF writes `u(t) = Kp e(t)` with nothing else, which is what you get with
feedforward **off** — the default here, so the sketch matches the sheet.

Press `w` to add the Activity 1 open-loop prediction underneath, so
`u = pwm_predicted(ωd) + Kp e`. The steady-state error collapses, because the
feedforward now supplies the standing output and the error no longer has to.

Run it both ways. The contrast explains what the integral term in Activity 4
actually buys you, and what it does not.

## Logging for the report

```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200 | tee activity3_kp.csv
```

Press `g`, then step through your `Kp` values. Columns:

```
t_ms,kp,setpoint_rpm,rpm_l,err_l,pwm_l,rpm_r,err_r,pwm_r
```

`analyze_step.py` in [Day3Activity5](../Day3Activity5/) reads a compatible
format if you want rise time and overshoot numbers for each gain.

## Expected trouble

**Below ~90 RPM the left wheel behaves badly.** Activity 1 showed it doesn't
break away until PWM ≈ 50 and stalls below ≈ 75 coming down, while the fitted
dead-band is only 9.6. With pure P and a small `Kp`, the controller cannot
generate enough PWM to break stiction, so the wheel sits still and the error
stays large forever. That is a real property of the machine, worth reporting
rather than tuning around.

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
