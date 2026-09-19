# Day 3 — Activity 2: Measure and Display Actual Velocity

Pick a desired wheel velocity, drive the wheels open-loop using the feedforward
model measured in [Activity 1](../Day3Activity1/), sample the encoders at a
fixed interval, and show target / actual / error live.

Still **open loop** — the commanded PWM never reacts to the measured speed. The
error column is the deliverable: it shows how far the Activity 1 model drifts
from reality, which is the argument for closing the loop later.

## Build and run

```bash
FQBN="esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled,CDCOnBoot=cdc"

arduino-cli compile --fqbn "$FQBN" Day3Activity2
arduino-cli upload -p /dev/ttyACM0 --fqbn "$FQBN" Day3Activity2
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

Lift the chassis so the wheels spin free.

## Commands

| Command | Effect |
| --- | --- |
| `v150` | both wheels to 150 RPM (negative reverses, e.g. `v-100`) |
| `m0.35` | both wheels to 0.35 m/s |
| `l150` / `r150` | one wheel only |
| `p200` | raw PWM 200, bypassing the feedforward model |
| `s` | stop |
| `f0.3` | filter strength, 0.01–1.0 (`f1` disables smoothing) |
| `t` | toggle live table ↔ CSV log |
| `h` | help |

## The live display

```
 wheel |  tgt rpm  act rpm  err rpm |  tgt m/s  act m/s  err m/s |  pwm
 ------+---------------------------+---------------------------+-----
 LEFT  |   150.0    147.3      2.7 |   0.338    0.332    0.006 |   97
 RIGHT |   150.0    151.1     -1.1 |   0.338    0.340   -0.002 |  101
 filter alpha 0.30 | feedforward | 't' for CSV, 'h' for help
```

The block repaints in place using ANSI cursor-up. If your terminal shows escape
codes as literal garbage, set `ansiRepaint = false` in the sketch, or press `t`
for CSV mode.

**Error is `target − actual`.** Positive means the wheel is running slower than
asked.

## How it measures

Encoders are sampled every **50 ms** (20 Hz); the terminal repaints every
250 ms. Decoupling the two keeps display work from slowing the measurement.

The sampling deadline advances by a fixed step rather than from "now":

```c
nextSample = nextSample + SAMPLE_INTERVAL_MS;
```

so serial traffic and repaints can't make the interval drift. Actual elapsed
time is still measured with `micros()` and used in the arithmetic, so a late
sample gives a correct speed rather than a wrong one.

Per sample, per wheel:

```
dCounts      = count - lastCount            (signed, x4 quadrature)
countsPerSec = dCounts * 1e6 / dMicros
RPM          = countsPerSec * 60 / COUNTS_PER_OUTPUT_REV
v (m/s)      = RPM / 60 * pi * D
error        = target - actual
```

The displayed speed is smoothed with an exponential moving average,
`rpmFilt += alpha * (rpmRaw - rpmFilt)`, default `alpha = 0.30`. At 50 ms and
840 counts/rev, one count of quantisation is about 1.4 RPM, so the raw signal
visibly jitters. Press `f1` to see it unfiltered — worth doing once so you know
what the filter is hiding. Filtering costs lag, which matters when you close the
loop.

## Logging

Press `t` for CSV mode, which prints one row per 50 ms sample:

```
t_ms,target_rpm_l,rpm_l,err_rpm_l,mps_l,pwm_l,target_rpm_r,rpm_r,err_rpm_r,mps_r,pwm_r
```

Capture it with:

```bash
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200 | tee activity2_log.csv
```

Use CSV mode rather than the table when piping to a file — the table's escape
codes make a mess of the log.

## Two constants to check

**`COUNTS_PER_OUTPUT_REV = 840`** — carried over from Activity 1 and **probably
wrong**. Activity 1 measured ~420 RPM at full duty on a 5 V rail, but these are
300 RPM / 6 V motors, so full duty should give *under* 300 RPM. If the encoder
is 11 PPR rather than the assumed 7, the true value is 1320 and everything
scales by 0.64. Calibrate: mark a wheel, turn it exactly 10 revolutions by hand,
`counts / 10` is the answer.

**`WHEEL_DIAMETER_MM = 43.0`** — an **assumption**, not a measurement. The wheel
size isn't recorded anywhere in this repository. Every m/s figure scales
directly with it; RPM figures don't depend on it at all. Measure your wheel and
correct it.

Both constants live at the top of the sketch. If you change
`COUNTS_PER_OUTPUT_REV`, re-run Activity 1 and paste the new feedforward
coefficients into `FF_SLOPE` / `FF_INTERCEPT` — they're expressed in the same
RPM units and won't survive a rescale.

## What to expect

- **Around 150–250 RPM the error should be small**, a few RPM, because that's
  the middle of Activity 1's linear region where the model was fitted.
- **Below ~90 RPM the left wheel misbehaves.** Activity 1 showed it doesn't
  break away until PWM ≈ 50 and stalls below ≈ 75 on the way down, while the
  fitted dead-band is only 9.6. Ask for `v40` and the left wheel may not turn at
  all while the right one does — the error column will show it plainly.
- **The two wheels differ by around 3%** at the same target, which is the gain
  mismatch from Activity 1.
- **Loading a wheel with your finger drops the actual speed and the PWM never
  responds.** That is the clearest demonstration of what open loop means, and
  it's worth capturing in CSV mode for the write-up.
