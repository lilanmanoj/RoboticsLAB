# Day 3 — Activity 1: Open-Loop Motor Characterization

Applies a sequence of PWM values to one wheel at a time, waits for the speed to
settle, measures the steady-state encoder speed, and repeats for the second
wheel. Output is CSV on the serial port; `plot_pwm_rpm.py` turns it into the
RPM-vs-PWM characteristic and fits the motor model.

## Files

| File | Purpose |
| --- | --- |
| `Day3Activity1.ino` | The sketch: PWM sweep + quadrature encoder measurement, CSV over Serial |
| `plot_pwm_rpm.py` | Plots RPM vs PWM and prints gain / dead-band / left-right mismatch |

## Hardware

Pin map is taken from the repository [README.md](../README.md).

| Signal | GPIO |
| --- | --- |
| PWMA / AIN1 / AIN2 (Motor A = Left) | 4 / 10 / 11 |
| PWMB / BIN1 / BIN2 (Motor B = Right) | 5 / 47 / 48 |
| STBY (driver enable) | 1 |
| Left encoder C1 / C2 | 38 / 39 |
| Right encoder C1 / C2 | 36 / 37 |

**Lift the chassis so both wheels spin free.** A wheel touching the bench adds a
load that changes the characteristic, and the robot will drive off the table at
PWM 255.

> **N16R8 caveat:** GPIO 35/36/37 are used by octal (OPI) PSRAM on
> ESP32-S3-WROOM-1 **N16R8** modules. Since the right encoder sits on GPIO 36/37,
> build with `PSRAM=disabled` (the default). If PSRAM is enabled the right
> encoder will read garbage or the board will not boot.

## Build, upload, capture

```bash
# one-time
arduino-cli core install esp32:esp32

# compile (plain S3 target)
arduino-cli compile --fqbn esp32:esp32:esp32s3 Day3Activity1

# compile for a 16MB N16R8 DevKitC-1, PSRAM off (see caveat above)
arduino-cli compile \
  --fqbn "esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled,CDCOnBoot=cdc" \
  Day3Activity1

# find the port, then upload
arduino-cli board list
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32s3 Day3Activity1

# capture the run into a CSV file (Ctrl-C when "# done." appears)
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200 | tee day3_activity1.csv
```

The sweep starts 3 s after boot. Send `r` on the serial port to repeat the whole
run without re-flashing (useful for a second data set to average).

Runtime is about `19 points × 2 sweeps × 2 motors × 2.1 s ≈ 2.7 minutes`.

## What the sketch does

1. Both motors off, driver enabled.
2. For the left motor: drive PWM = 0, 20, 30, … 255, then back down 255 … 0.
   At each value it waits `SETTLE_MS` (1200 ms) for the speed to settle, then
   takes 3 back-to-back 300 ms encoder windows and averages them.
3. Repeat the whole thing for the right motor.
4. Each measurement is printed as one CSV row.

Encoder counting is x4 quadrature: both channels are attached on `CHANGE` and
decoded through a state-transition table, so every edge counts and the count is
signed (direction-aware).

## CSV columns

| Column | Meaning |
| --- | --- |
| `motor` | `LEFT` or `RIGHT` |
| `direction` | `FWD` / `REV` drive direction |
| `sweep` | `UP` (0→255) or `DOWN` (255→0) |
| `pwm` | commanded PWM, 0–255 |
| `duty_percent` | same value as a duty-cycle percentage |
| `counts` | signed encoder counts accumulated over all windows |
| `window_ms` | total measurement time for the point |
| `counts_per_sec` | raw encoder rate |
| `rpm_output` | **wheel / gearbox output shaft RPM — this is the y-axis** |
| `rpm_motor` | motor shaft RPM (= `rpm_output × GEAR_RATIO`) |
| `rpm_spread` | max−min of the 3 sub-window RPMs; a *quality* figure |

`rpm_spread` is the honesty check on "steady state". If it is a few percent of
`rpm_output`, the motor had settled. If it is large, increase `SETTLE_MS`.

## Calibration you must confirm

The RPM numbers are only as good as this constant near the top of the sketch:

```
COUNTS_PER_OUTPUT_REV = ENCODER_PPR_PER_CHANNEL (7) × QUADRATURE (4) × GEAR_RATIO (30) = 840
```

The 7 PPR and the 1:30 gearbox are the usual values for an N20 6V/300RPM with a
hall encoder, but gearbox ratios vary between batches. To verify: mark the wheel,
run the sketch, and while it is idle turn the wheel by hand exactly 10 full
revolutions — `counts/10` is your true `COUNTS_PER_OUTPUT_REV`. A wrong value
scales the whole curve by a constant; it does not change its shape.

If a motor reports negative RPM while physically driving forward, set
`LEFT_ENCODER_SIGN` / `RIGHT_ENCODER_SIGN` to `-1` (motor leads or encoder
channels swapped).

## Plotting

```bash
pip install matplotlib numpy
python3 plot_pwm_rpm.py day3_activity1.csv --out pwm_rpm.png
```

Options: `--motor-shaft` plots motor-shaft RPM instead of wheel RPM,
`--direction REV` plots the reverse sweep (enable `TEST_REVERSE_DIRECTION` in the
sketch first).

Plot conventions used, and worth keeping if you plot by hand instead:

- **x = PWM (0–255), y = RPM.** PWM is what you commanded, so it is the
  independent variable and belongs on x.
- **One y-axis.** Never put left RPM and right RPM on two different scales — the
  point of the plot is that they are directly comparable.
- Left = blue, right = orange, held constant across every plot in the report.
- Up sweep solid + circles, down sweep dashed + squares, so the two are
  distinguishable without relying on colour.
- Start the y-axis at 0. A truncated axis exaggerates the left/right mismatch.
- Overlay the fitted straight line on the linear region only — not through the
  dead-band, where the model does not hold.

The script also prints the fitted model (numbers below are from a synthetic test
file, not real hardware — yours will differ):

```
motor    gain [RPM/PWM]   deadband PWM   RPM @ 255  max |spread|
LEFT             1.4675           41.6       313.1          1.50
RIGHT            1.3424           48.2       277.6          1.48

Left/right gain mismatch: 8.5%
```

## Reading the graph

The curve has three regions:

1. **Dead-band (PWM ≈ 0 → 40-60):** RPM stays at 0. The applied voltage cannot
   overcome static friction plus the TB6612FNG's internal voltage drop. Where the
   fitted line crosses zero RPM is the dead-band PWM. This is the single most
   useful number from the activity — any closed-loop controller written later
   must add this offset, or its integral term will wind up doing nothing at low
   commands.
2. **Linear region:** RPM rises roughly proportionally to PWM. The slope is the
   open-loop plant gain **K [RPM per PWM count]**. Model:

   ```
   RPM ≈ K · (PWM − PWM_deadband)
   ```

   Inverting it gives the feedforward term for Activity 2/3:

   ```
   PWM = RPM_target / K + PWM_deadband
   ```

3. **Saturation / droop at the top:** the curve flattens as the motor approaches
   its no-load speed and as the battery sags under current draw. If the top is
   badly bent, check the 18650 voltage — a dropping supply looks exactly like
   motor saturation.

Things to look for and write up:

- **Left vs right mismatch.** The two gains will differ by a few percent even for
  identical part numbers. That difference is precisely why the robot curves when
  both wheels get the same PWM, and it motivates closed-loop speed control.
- **Hysteresis.** If the down sweep sits above the up sweep, the motor is warm /
  the grease has loosened / stiction is releasing differently. A visible gap means
  the plant is not a pure static map, and the open-loop model has a limit.
- **Dead-band asymmetry** between the two motors — one may start moving several
  PWM counts before the other.
- **`rpm_spread` size.** Report it as your measurement uncertainty rather than
  quoting RPM to three decimals.

## Tuning knobs in the sketch

| Constant | Default | Effect |
| --- | --- | --- |
| `PWM_STEPS[]` | 19 values | denser near the bottom to resolve the dead-band |
| `SETTLE_MS` | 1200 | raise if `rpm_spread` is large |
| `SAMPLE_MS` / `SAMPLES_PER_PT` | 300 / 3 | longer windows = less quantization noise |
| `DO_DOWN_SWEEP` | `true` | set `false` to halve runtime, loses hysteresis data |
| `TEST_REVERSE_DIRECTION` | `false` | set `true` to also characterize reverse |
| `PWM_FREQ_HZ` | 20000 | above audible; low frequencies change the dead-band |
