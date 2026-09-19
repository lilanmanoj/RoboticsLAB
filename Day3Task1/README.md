# Task 1 — Calculate Motion Toward a Target

Lab PDF p.32.

```
ρ  = √[(xd − x)² + (yd − y)²]
θd = atan2(yd − y, xd − x)
eθ = θd − θ
v  = Kv ρ      (saturated at v_max)
ω  = Kω eθ     (saturated at ω_max)
```

Pure computation — **no motors, no encoders**. Prints the worked answer for the
lab-sheet values at boot, then recomputes for any pose and target you type, so
it doubles as a checker for your hand calculations.

## Build and run

```bash
FQBN="esp32:esp32:esp32s3:FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,PSRAM=disabled,CDCOnBoot=cdc"

arduino-cli compile --fqbn "$FQBN" Day3Task1
arduino-cli upload -p /dev/ttyACM0 --fqbn "$FQBN" Day3Task1
arduino-cli monitor -p /dev/ttyACM0 -c baudrate=115200
```

No hardware needed beyond the board — safe to run with the robot on the desk.

## Given values (from the lab sheet)

| Quantity | Value |
| --- | --- |
| Initial pose `(x, y, θ)` | `(0, 0, 0°)` |
| Target `(xd, yd)` | `(1.5 m, 1.0 m)` |
| `Kv` | 0.5 s⁻¹ |
| `Kω` | 2.0 s⁻¹ |
| `v_max` | 0.25 m/s |
| `ω_max` | 1.5 rad/s |

## The answer

```
ρ       = √(1.5² + 1.0²)          = 1.8028 m
θd      = atan2(1.0, 1.5)          = 0.5880 rad = 33.69°
eθ      = 0.5880 − 0               = 0.5880 rad = 33.69°
v = Kv ρ = 0.5 × 1.8028 = 0.9014 m/s   → 0.2500 m/s   [SATURATED]
ω = Kω eθ = 2.0 × 0.5880 = 1.1760 rad/s → 1.1760 rad/s  (within limit)
```

**`v` saturates and `ω` does not.** `Kv ρ` comes out at 0.90 m/s, well over the
0.25 m/s limit, so the commanded linear velocity is clamped. `Kω eθ` lands at
1.18 rad/s against a 1.5 limit, so it passes through untouched. The sketch
flags each with `[SATURATED]` so you can see which is which.

## Commands

| Command | Effect |
| --- | --- |
| `p0,0,0` | set robot pose `x, y, θ°` |
| `t1.5,1.0` | set target `xd, yd` |
| `r` | recompute and print |
| `h` | help |

Useful checks to try:

```
p0,0,90      <- robot facing +y: eθ becomes negative, ω turns it right
t-1.5,1.0    <- target behind-left: θd goes past 90°
t0.1,0.0     <- close target: v no longer saturates
```

## Angle wrapping

`wrapPi()` folds every heading error into `(−π, π]`:

```c
float wrapPi(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a <= -PI) a += 2.0f * PI;
  return a;
}
```

Without it a heading error of +350° would be acted on as a long way round to
the left instead of a short hop to the right. It makes no difference to the
lab-sheet numbers — `eθ` is only 33.69° — but try `p0,0,-170` with a target
behind the robot and it matters immediately. Every heading error in a real
controller has to be wrapped.

## Where this goes next

`v` and `ω` from here are the input to [Task 2](../Day3Task2/), which converts
them to wheel velocities. [Task 4](../Day3Task4/) runs this same arithmetic at
50 Hz against a live odometry estimate.
