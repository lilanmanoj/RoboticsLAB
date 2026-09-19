# Line Following Robot — 5-bit IR Array + PID

Lab PDF p.38, Activity 1 of the line-following section:

> Use a 5-bit IR sensor array to design a line following robot that follows a
> given line with a PID controller.

Two nested loops:

- **outer** — PID on the line position error from the MD0482 array, producing a
  differential RPM correction
- **inner** — the Activity 4 PI wheel velocity controllers, which execute
  `base ± correction` on each wheel

Steering through velocity control rather than straight to PWM means the robot
follows the line the same way on a fresh battery as on a flat one, and the 3.3%
left/right gain mismatch from Activity 1 stops biasing the steering.

## ⚠ Wire the sensor first — the pins are a guess

**The 5-bit IR array pins are not in the repository [README.md](../README.md).**
I picked five free GPIOs, avoiding the strapping pins (0/3/45/46), the USB D+/D−
pair (19/20), and everything the drivetrain already uses:

| Sensor | GPIO |
| --- | --- |
| D0 (leftmost) | 6 |
| D1 | 7 |
| D2 (centre) | 15 |
| D3 | 16 |
| D4 (rightmost) | 17 |

Plus module `VCC` → 3.3 V and `GND` → common ground. The module runs on 3.3–5 V
and outputs digital TTL; **power it from 3.3 V** so its outputs stay inside the
ESP32's input range.

Change `IR_PIN[]` at the top of the sketch to match how you actually wire it.

## Build and run

```bash
FQBN="esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled,CDCOnBoot=cdc"

arduino-cli compile --fqbn "$FQBN" Day3LineFollower
arduino-cli upload -p /dev/ttyACM0 --fqbn "$FQBN" Day3LineFollower
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

## Step 1 — verify the sensors before driving anywhere

Type `t` for sensor test mode. Motors stay off; the display shows the five bits
live:

```
 sensors 11011   on line 1   error   0.00
```

Bits are printed in the lab sheet's convention — **`0` means that sensor is
over the line**. Slide the array across the line and check against PDF p.36:

| Reading | Meaning |
| --- | --- |
| `01111` | line off to the left |
| `10111` | line slightly to the left |
| `11011` | line centred |
| `11101` | line slightly to the right |
| `11110` | line off to the right |

Three things to confirm:

1. **All five sensors respond.** A dead bit is a wiring fault.
2. **The order is left-to-right.** If moving the array left lights up the wrong
   end, your `IR_PIN[]` order is reversed.
3. **The polarity is right.** If the bits read `00100` on the line instead of
   `11011`, set `IR_ACTIVE_LOW = false`. This is also what you change for a
   white line on a black background.

Adjust the module's sensitivity potentiometers until the transition is crisp.
Getting this right is most of the work — no amount of PID tuning fixes a sensor
that cannot see the line.

Press `t` again to leave test mode.

## Step 2 — follow the line

Place the robot on the line and send `G`. Send `s` to stop.

| Command | Effect |
| --- | --- |
| `G` | go / start following |
| `s` | stop |
| `b120` | base cruise speed, RPM |
| `p28` | steering `Kp` |
| `i0` | steering `Ki` |
| `d12` | steering `Kd` |
| `m120` | max differential correction, RPM |
| `P0.35` / `I2.5` | inner wheel velocity gains |
| `t` | sensor test mode |
| `g` | toggle CSV logging |
| `h` | help |

## Live display

```
 RUN     sensors 11011  on line 1  base 120 rpm
 steering  err   0.00  I    0.00  D     0.00  -> corr     0.0 rpm
 gains     Kp 28.0  Ki 0.00  Kd 12.0
 LEFT   set   120.0  act   119.8 rpm  pwm    82
 RIGHT  set   120.0  act   120.1 rpm  pwm    83
```

## How the error is computed

`computeLineError()` takes the **weighted average** of the positions of every
sensor currently seeing the line, with weights `−2, −1, 0, +1, +2`:

```c
err = sum(weight[i] for triggered i) / number_triggered;
```

With two adjacent sensors triggered the error lands halfway between them, so
the error is continuous even though the sensors are discrete. A
highest-priority-sensor lookup table would quantise the error to five values
and make the derivative term useless — it would only ever see jumps.

Positive error means the line is to the **right** of centre, so the robot must
turn right: speed up the left wheel, slow the right one.

## Tuning

Start with the defaults (`Kp 28`, `Ki 0`, `Kd 12`, base 120 RPM):

| Symptom | Fix |
| --- | --- |
| drifts off on gentle curves | raise `Kp` |
| oscillates / weaves down a straight | lower `Kp`, or raise `Kd` |
| overshoots each correction then wobbles | raise `Kd` |
| jittery, twitchy steering | lower `Kd` — it amplifies sensor noise |
| cuts corners on tight turns | lower `b` (base speed) |
| too slow to be interesting | raise `b`, then re-tune `Kp` |

**Tune at the speed you intend to run.** `Kp` and base speed interact strongly:
gains that work at 80 RPM will weave badly at 200.

**Leave `Ki` at 0 unless you have a reason.** A line follower has no standing
offset to integrate away — the line is where it is. `Ki` mostly adds wobble and
a lag after each corner. It's exposed so you can demonstrate that.

## Losing the line

If no sensor sees the line, the sketch keeps steering hard in the direction the
line was last seen, so the robot sweeps back onto it rather than driving
straight off the track. After `LOST_GIVEUP_MS` (1.2 s) it stops and prints
`# line lost - stopping` rather than continuing blind.

## Logging

Press `g`, then:

```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200 | tee line_log.csv
```

Columns:

```
t_ms,bits,on_line,line_err,integral,deriv,correction,
set_rpm_l,set_rpm_r,rpm_l,rpm_r,pwm_l,pwm_r
```

Plotting `line_err` against time is the figure for this activity: a well-tuned
run stays near zero with small excursions at corners, an under-damped one
oscillates continuously.

## Expected trouble

**Surface and lighting matter more than gains.** IR reflectance sensors are
sensitive to ambient light, sensor height above the surface, and how matte the
black line is. Re-run `t` and re-adjust the potentiometers whenever you change
venue.

**Sharp corners are a sensor-geometry problem, not a tuning problem.** If the
line leaves the 5-sensor span faster than the robot can turn, no gain fixes it —
you have to slow down.

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
