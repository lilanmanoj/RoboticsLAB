# Task 3 — Convert Wheel Velocity to RPM

Lab PDF p.34.

```
ω_wheel = v_wheel / r
RPM     = ω_wheel × 60 / (2π)
```

Converts `vL` and `vR` from [Task 2](../Day3Task2/) into the RPM setpoints that
feed the closed-loop wheel velocity controllers of Activities 4–6.

Pure computation — **no motors, no encoders**.

## Build and run

```bash
FQBN="esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled,CDCOnBoot=cdc"

arduino-cli compile --fqbn "$FQBN" Day3Task3
arduino-cli upload -p /dev/ttyACM0 --fqbn "$FQBN" Day3Task3
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

## The answer, continuing from Task 2

Inputs `vL = 0.1912 m/s`, `vR = 0.3088 m/s`, wheel radius `r = 0.0215 m`:

```
LEFT    ω_wheel = 0.1912 / 0.0215 =  8.8930 rad/s
        RPM     = 8.8930 × 9.5493 =   84.92 rpm

RIGHT   ω_wheel = 0.3088 / 0.0215 = 14.3628 rad/s
        RPM     = 14.3628 × 9.5493 = 137.15 rpm
```

**`RPM_L = 84.92`, `RPM_R = 137.15`.**

The factor `60/(2π) = 9.5493` converts rad/s to rev/min.

## Commands

| Command | Effect |
| --- | --- |
| `c0.1912,0.3088` | set wheel velocities `vL, vR` in m/s |
| `d0.043` | set wheel diameter, metres |
| `n85,137` | **inverse**: RPM → m/s, useful for checking encoder readings |
| `r` | recompute and print |
| `h` | help |

The `n` command is handy when you have a measured RPM from Activity 2 or 6 and
want to know what wheel speed it corresponds to.

## Wheel radius

**`wheelDiameterM = 0.043` is an assumption** — not recorded anywhere in the
repository. Measure yours.

RPM scales **inversely** with radius. A wheel 10% larger than assumed means the
true RPM needed is 10% *lower* than this task reports, and the robot will
overshoot every distance you command. Of the three unverified constants in this
lab, this is the one that most directly corrupts the position control in
[Task 4](../Day3Task4/).

## A sanity check worth doing

The sketch flags any result above `RPM_MAX = 400` with `[ABOVE MEASURED MAX]`.
That threshold comes from [Activity 1](../Day3Activity1/), where the wheels
reached about 420 RPM at full duty.

If a task asks for a wheel speed above that, no controller can deliver it — the
demand has to be saturated back in Task 2 instead. `84.92` and `137.15` are both
comfortably achievable, and sit in the middle of the linear region where the
Activity 1 feedforward model is accurate.

Note that `RPM_MAX` inherits the unverified `COUNTS_PER_OUTPUT_REV` from
Activity 1, so if you recalibrate that constant this threshold moves too.

## Where this goes next

These RPM values are the setpoints for the wheel velocity controllers:
[Activity 4](../Day3Activity4/) for a single wheel, [Activity 6](../Day3Activity6/)
for both independently. [Task 4](../Day3Task4/) runs the whole Task 1 → 2 → 3
chain at 50 Hz.
