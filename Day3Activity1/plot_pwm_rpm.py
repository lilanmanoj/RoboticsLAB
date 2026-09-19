#!/usr/bin/env python3
"""
plot_pwm_rpm.py - plot the PWM-RPM characteristic captured by Day3Activity1.ino

Usage:
    python3 plot_pwm_rpm.py day3_activity1.csv
    python3 plot_pwm_rpm.py day3_activity1.csv --out pwm_rpm.png

Reads the CSV that the sketch streams over Serial ('#' comment lines and any
serial noise are ignored), plots RPM vs PWM for both motors, and prints the
fitted straight-line model of each motor's linear region:

    RPM = gain * (PWM - deadband_pwm)      for PWM > deadband_pwm

Requires: matplotlib, numpy   ->   pip install matplotlib numpy
"""

import argparse
import csv
import sys
from collections import defaultdict

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Categorical palette, fixed slot order (slot 1 blue, slot 2 orange).
SERIES_COLORS = {"LEFT": "#2a78d6", "RIGHT": "#eb6834"}
SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e3e2de"

# Sweep direction is encoded by line style as well as by color, so the two
# sweeps stay distinguishable in greyscale / for colorblind readers.
SWEEP_STYLES = {"UP": ("-", "o"), "DOWN": ("--", "s")}

COLUMNS = [
    "motor", "direction", "sweep", "pwm", "duty_percent", "counts",
    "window_ms", "counts_per_sec", "rpm_output", "rpm_motor", "rpm_spread",
]


def load_rows(path):
    """Parse the sketch's CSV, tolerating comment lines and partial rows."""
    rows = []
    with open(path, newline="", errors="replace") as fh:
        for raw in fh:
            line = raw.strip()
            if not line or line.startswith("#") or line.startswith("motor,"):
                continue
            parts = next(csv.reader([line]))
            if len(parts) != len(COLUMNS):
                continue
            try:
                row = dict(zip(COLUMNS, parts))
                row["pwm"] = int(row["pwm"])
                row["rpm_output"] = float(row["rpm_output"])
                row["rpm_motor"] = float(row["rpm_motor"])
                row["rpm_spread"] = float(row["rpm_spread"])
            except ValueError:
                continue
            rows.append(row)
    return rows


def fit_linear_region(pwm, rpm, min_rpm_frac=0.15):
    """
    Least-squares fit over the points that are actually moving, i.e. above
    min_rpm_frac of the maximum speed. Returns (gain, intercept, deadband).

    gain     : RPM per PWM count, the open-loop plant gain
    deadband : PWM at which the fitted line crosses zero RPM (stiction +
               driver drop; below it the wheel does not turn)
    """
    pwm = np.asarray(pwm, dtype=float)
    rpm = np.asarray(rpm, dtype=float)
    peak = np.max(np.abs(rpm)) if rpm.size else 0.0
    if peak <= 0:
        return None
    mask = np.abs(rpm) > min_rpm_frac * peak
    if mask.sum() < 2:
        return None
    gain, intercept = np.polyfit(pwm[mask], rpm[mask], 1)
    deadband = -intercept / gain if gain != 0 else float("nan")
    return gain, intercept, deadband


def write_table(path, series, pwm_values):
    """
    Write the plotted curve itself as a tidy CSV: one row per PWM value with
    each motor's up-sweep, down-sweep, and mean RPM. This is the graph as
    numbers - the up/down columns are kept because the two differ inside the
    dead-band, where averaging them would hide the hysteresis.
    """
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["pwm", "left_rpm_up", "left_rpm_down", "left_rpm_mean",
                    "right_rpm_up", "right_rpm_down", "right_rpm_mean"])
        for pwm in pwm_values:
            row = [pwm]
            for motor in ("LEFT", "RIGHT"):
                vals = []
                for sweep in ("UP", "DOWN"):
                    pts = series.get((motor, sweep), {}).get(pwm)
                    vals.append(float(np.mean(pts)) if pts else None)
                mean = ([v for v in vals if v is not None] or [None])
                mean = float(np.mean(mean)) if mean[0] is not None else None
                row += [f"{v:.3f}" if v is not None else "" for v in vals]
                row.append(f"{mean:.3f}" if mean is not None else "")
            w.writerow(row)
    print(f"wrote {path}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="CSV captured from the serial port")
    ap.add_argument("--out", default="pwm_rpm.png", help="output image path")
    ap.add_argument("--table", default="pwm_rpm_table.csv",
                    help="write the plotted curve as a CSV table")
    ap.add_argument("--motor-shaft", action="store_true",
                    help="plot motor-shaft RPM instead of output/wheel RPM")
    ap.add_argument("--direction", default="FWD",
                    help="which drive direction to plot (default FWD)")
    args = ap.parse_args()

    rows = [r for r in load_rows(args.csv) if r["direction"] == args.direction]
    if not rows:
        sys.exit(f"no data rows for direction {args.direction} in {args.csv}")

    key = "rpm_motor" if args.motor_shaft else "rpm_output"
    ylabel = "Motor-shaft speed (RPM)" if args.motor_shaft else "Wheel speed (RPM)"

    # series[(motor, sweep)] -> {pwm: rpm}, averaged if a point repeats
    series = defaultdict(lambda: defaultdict(list))
    for r in rows:
        series[(r["motor"], r["sweep"])][r["pwm"]].append(r[key])

    if args.table:
        all_pwm = sorted({p for pts in series.values() for p in pts})
        write_table(args.table, series, all_pwm)

    fig, ax = plt.subplots(figsize=(8.5, 5.5), dpi=160)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)

    for motor in ("LEFT", "RIGHT"):
        for sweep in ("UP", "DOWN"):
            pts = series.get((motor, sweep))
            if not pts:
                continue
            pwm = sorted(pts)
            rpm = [float(np.mean(pts[p])) for p in pwm]
            style, marker = SWEEP_STYLES[sweep]
            ax.plot(pwm, rpm,
                    linestyle=style, marker=marker, linewidth=2,
                    markersize=5, markeredgecolor=SURFACE, markeredgewidth=1.2,
                    color=SERIES_COLORS[motor],
                    label=f"{motor.title()} motor - {sweep.lower()} sweep")

    # Fit each motor over all of its sweeps combined.
    print(f"\nLinear-region fit ({ylabel}, direction {args.direction})")
    print(f"{'motor':<7}{'gain [RPM/PWM]':>16}{'deadband PWM':>15}"
          f"{'RPM @ 255':>12}{'max |spread|':>14}")
    fit_lines = {}
    for motor in ("LEFT", "RIGHT"):
        pwm_all, rpm_all, spread_all = [], [], []
        for sweep in ("UP", "DOWN"):
            pts = series.get((motor, sweep), {})
            for p, vals in pts.items():
                pwm_all.append(p)
                rpm_all.append(float(np.mean(vals)))
        for r in rows:
            if r["motor"] == motor:
                spread_all.append(r["rpm_spread"])
        if not pwm_all:
            continue
        fit = fit_linear_region(pwm_all, rpm_all)
        if fit is None:
            print(f"{motor:<7}{'no motion detected':>16}")
            continue
        gain, intercept, deadband = fit
        fit_lines[motor] = fit
        top = gain * 255 + intercept
        print(f"{motor:<7}{gain:>16.4f}{deadband:>15.1f}{top:>12.1f}"
              f"{max(spread_all):>14.2f}")

        xs = np.array([deadband, 255.0])
        ax.plot(xs, gain * xs + intercept, linewidth=1, alpha=0.55,
                color=SERIES_COLORS[motor], zorder=1)

    if len(fit_lines) == 2:
        gl, gr = fit_lines["LEFT"][0], fit_lines["RIGHT"][0]
        mismatch = 100.0 * abs(gl - gr) / max(abs(gl), abs(gr))
        print(f"\nLeft/right gain mismatch: {mismatch:.1f}% "
              f"(this is why the robot veers under equal PWM)")
        print("Feedforward for a target wheel speed w (RPM):")
        for motor, (gain, intercept, deadband) in fit_lines.items():
            print(f"  pwm_{motor.lower():<5} = {1.0/gain:6.3f} * w + {deadband:5.1f}")

    ax.set_xlabel("PWM duty (0-255)", color=TEXT_SECONDARY)
    ax.set_ylabel(ylabel, color=TEXT_SECONDARY)
    ax.set_title("Open-loop PWM-RPM characteristic", color=TEXT_PRIMARY,
                 fontsize=13, pad=12, loc="left")
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=TEXT_SECONDARY)
    ax.set_xlim(0, 260)
    ax.axhline(0, color=GRID, linewidth=1)
    ax.legend(frameon=False, labelcolor=TEXT_SECONDARY, fontsize=9)

    fig.tight_layout()
    fig.savefig(args.out, facecolor=SURFACE)
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
