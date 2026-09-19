# Task 2 — Convert Robot Velocity to Wheel Velocities

Lab PDF p.33.

```
vR = v + (L/2) ω
vL = v − (L/2) ω
```

Takes `v` and `ω` from [Task 1](../Day3Task1/), applies the differential-drive
inverse kinematics, checks the wheel velocities against the permitted limit and
applies saturation if required.

Pure computation — **no motors, no encoders**.

## Build and run

```bash
FQBN="esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled,CDCOnBoot=cdc"

arduino-cli compile --fqbn "$FQBN" Day3Task2
arduino-cli upload -p /dev/ttyACM0 --fqbn "$FQBN" Day3Task2
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

## The answer, continuing from Task 1

Inputs `v = 0.2500 m/s`, `ω = 1.1760 rad/s`, `L = 0.100 m`:

```
vR = 0.2500 + 0.0500 × 1.1760 = 0.3088 m/s
vL = 0.2500 − 0.0500 × 1.1760 = 0.1912 m/s
```

Peak is 0.3088 m/s against a 0.60 m/s limit — **no saturation required** for
the lab-sheet numbers.

To see saturation actually engage, try `c0.5,2.0`:

```
vR = 0.5 + 0.05×2.0 = 0.600 m/s
vL = 0.5 − 0.05×2.0 = 0.400 m/s     <- peak 0.600, exactly at the limit
```

or `c0.6,3.0` which exceeds it and triggers the scaling.

## Commands

| Command | Effect |
| --- | --- |
| `c0.25,1.176` | set robot velocity `v, ω` |
| `L0.10` | set wheel base, metres |
| `m0.60` | set the permitted wheel speed limit |
| `r` | recompute and print |
| `h` | help |

## Saturation by scaling, not clipping

When the peak exceeds the limit, **both** wheels are scaled by the same factor:

```c
const float scale = vWheelMax / peak;
vL = vLraw * scale;
vR = vRraw * scale;
```

Clipping each wheel independently would change the *difference* between them.
Since `ω = (vR − vL)/L`, that changes the angular velocity — you would ask for a
gentle arc and get a sharp one, or vice versa. Scaling preserves the ratio, so
the **path stays the same shape** and only the speed along it drops.

The sketch prints the effective `v` and `ω` after scaling so you can confirm:
`v` drops, `ω` is unchanged.

This is the same policy used in [Activity 7](../Day3Activity7/) and
[Task 4](../Day3Task4/), where it runs live.

## Wheel base

**`L = 0.100 m` is an assumption.** It is not recorded anywhere in the
repository. Measure yours centre-to-centre between the wheel contact patches
and set it with `L<value>`, or edit `wheelBaseL` at the top of the sketch.

`L` scales the entire turning half of the equation. A 10% error in `L` is a 10%
error in the differential term, and the robot will consistently under- or
over-turn.

## Where this goes next

`vL` and `vR` from here are the input to [Task 3](../Day3Task3/), which converts
them to the RPM setpoints for the wheel velocity controllers.
