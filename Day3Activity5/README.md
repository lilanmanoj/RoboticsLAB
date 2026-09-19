# Activity 5 — Step Response of the Wheel Velocity Controller

Lab PDF p.26. Commands a sequence of desired wheel velocities, records desired
and actual RPM at every control cycle, and extracts rise time, settling time,
overshoot and steady-state error.

## Files

| File | Purpose |
| --- | --- |
| `Day3Activity5.ino` | runs the step sequence, streams one CSV row per control cycle |
| `analyze_step.py` | plots the response and computes the four metrics per step |
| `RobotBase.h`, `VelocityPI.h` | shared drivetrain core and PI controller |

## Build and run

```bash
FQBN="esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled,CDCOnBoot=cdc"

arduino-cli compile --fqbn "$FQBN" Day3Activity5
arduino-cli upload -p /dev/ttyACM0 --fqbn "$FQBN" Day3Activity5
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200 | tee step_log.csv
```

Lift the chassis. Then type `r` and wait — the sequence takes 17.5 s.

## Commands

| Command | Effect |
| --- | --- |
| `r` | run the step sequence once, streaming CSV |
| `k0.5` | set `Kp` before the run |
| `i2.5` | set `Ki` before the run (`i0` = pure P) |
| `s` | stop immediately |
| `h` | help |

## The step sequence

```
0 → 150 → 0 → 250 → 100 → 250 → 0 RPM,  2.5 s each
```

It starts and ends at zero so the fall response is captured too, and includes
both a large step (`0→250`) and a small one (`100→250`) — the controller does
not behave the same on both, because the large one saturates the PWM and the
small one does not.

The `100→250` and `250→100` pair gives you rise and fall from the same
operating point, which is the cleanest comparison for the report.

Edit `STEP_RPM[]` and `STEP_HOLD_MS` at the top of the sketch to change it.

Control loop runs at **50 Hz**, and one CSV row is emitted per cycle — so the
time resolution of your rise-time measurement is 20 ms.

## Analysis

```bash
pip install matplotlib numpy      # or: sudo apt install python3-matplotlib python3-numpy
python3 analyze_step.py step_log.csv --out step_response.png
```

Output:

```
step     from       to  wheel      rise    settle  overshoot   ss error     final
--------------------------------------------------------------------------------
   1        0      150   left   0.380 s   1.380 s      1.7 %       -0.7     150.7
   1        0      150  right   0.460 s   2.419 s      2.6 %       -0.6     150.6
   3        0      250   left   0.400 s   0.620 s      0.9 %       -0.4     250.4
```

Options: `--wheel left` / `--wheel right` to plot just one.

### Definitions used

| Metric | Definition |
| --- | --- |
| Rise time | 10% → 90% of the commanded step change |
| Settling time | time to enter and *stay* inside ±2% of the final value, from the step instant |
| Overshoot | `100 × (peak − final) / step_change`, 0 if never exceeded |
| Steady-state error | `setpoint − mean(actual over the last 25% of the step)` |

Two choices worth knowing about when you write these up:

**Overshoot is measured against the achieved final value, not the setpoint.** A
controller with a standing steady-state error isn't credited with overshoot it
never had.

**The settling band is 2% of the final value, but never narrower than 2% of the
step size.** Without that floor, a step back down to 0 RPM would get a band of
almost zero width and nothing would ever register as settled.

A metric that cannot be determined from the captured window prints as `—`
rather than a guess. If settling time is `—`, the response never stayed inside
the band before the step ended — extend `STEP_HOLD_MS` or fix the tuning.

## Suggested runs for the report

Capture the same sequence at several tunings and compare the tables:

```
k0.35  i0     r      <- pure P: expect standing error, little overshoot
k0.35  i2.5   r      <- PI: error to zero, some overshoot
k0.35  i8     r      <- too much I: overshoot and slow settling
k1.5   i2.5   r      <- high P: fast rise, oscillation
```

Save each to its own CSV so `analyze_step.py` can be run per tuning.

## Expected trouble

**The `0→250` step will saturate.** 250 RPM needs roughly PWM 160, but the
transient demand at the step instant exceeds 255, so the output rails briefly.
That is why the anti-windup guard in `VelocityPI.h` matters here — without it,
the `250→100` step that follows would be badly delayed.

**Steps involving 100 RPM may show a slow or absent left-wheel response.**
Activity 1 measured the left wheel stalling below PWM ≈ 75. At the bottom of
the range the plant is not the linear system these metrics assume, so the
numbers there describe stiction rather than controller tuning.

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
