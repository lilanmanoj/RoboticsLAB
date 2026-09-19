#!/usr/bin/env python3
"""
analyze_kp.py - extract the steady-state error vs Kp relationship for Activity 3
and compare it against the plant model measured in Activity 1.

Reads EITHER format:
  * the 50 Hz CSV log written when you press 'g' in the sketch
  * a capture of the 4 Hz live ANSI table (what you get if you forget 'g')

Table captures are enough for steady-state error but NOT for rise time or
overshoot - at 250 ms per frame the transient is only a couple of samples long.
Re-run with 'g' pressed for those.

Usage:
    python3 analyze_kp.py activity3_kp_*.csv
    python3 analyze_kp.py activity3_kp_*.csv --setpoint 150 --out kp_error.png

Requires: matplotlib, numpy   ->   pip install matplotlib numpy
"""

import argparse
import csv
import re
import statistics as st
import sys
from collections import defaultdict

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Plant model from Activity 1 (day3_activity1.csv):  RPM = K * (PWM - PWM_db)
PLANT_GAIN = 1.7096      # RPM per PWM count
PLANT_DEADBAND = 9.6     # PWM at which the fitted line crosses zero RPM

WHEEL_COLOR = {"LEFT": "#2a78d6", "RIGHT": "#eb6834"}
THEORY_COLOR = "#52514e"
SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e3e2de"

FRAME_RE = re.compile(
    r"Kp (\d+\.\d+)\s+feedforward (\w+)\s+setpoint\s+(-?\d+\.\d+) rpm")
ROW_RE = re.compile(
    r"(LEFT|RIGHT)\s*\|\s*(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s*\|\s*(-?\d+)")


def predicted_error(kp, setpoint):
    """
    Steady-state error of a P controller driving the Activity 1 plant.

    At equilibrium the PWM is Kp*e and the plant gives
        rpm = K*(Kp*e - PWM_db),   e = setpoint - rpm
    Solving for e:
        e = (setpoint + K*PWM_db) / (1 + Kp*K)

    Note it tends to zero only as Kp -> infinity. That is the structural
    limitation of P control, not a tuning failure.
    """
    return (setpoint + PLANT_GAIN * PLANT_DEADBAND) / (1.0 + kp * PLANT_GAIN)


def read_table_capture(path, setpoint):
    """Parse a capture of the live ANSI display. Yields (kp, wheel, rpm, pwm)."""
    kp = sp = None
    for line in open(path, errors="replace"):
        m = FRAME_RE.search(line)
        if m:
            kp, sp = float(m.group(1)), float(m.group(3))
            continue
        r = ROW_RE.search(line)
        if r and kp is not None and sp == setpoint:
            yield kp, r.group(1), float(r.group(3)), int(r.group(5))


def read_csv_log(path, setpoint):
    """Parse the 50 Hz CSV written in 'g' mode. Yields (kp, wheel, rpm, pwm)."""
    with open(path, newline="", errors="replace") as fh:
        rdr = csv.DictReader(
            (ln for ln in fh if ln.strip() and not ln.startswith("#")))
        for row in rdr:
            try:
                kp = float(row["kp"])
                if float(row["setpoint_rpm"]) != setpoint:
                    continue
                yield kp, "LEFT", float(row["rpm_l"]), int(row["pwm_l"])
                yield kp, "RIGHT", float(row["rpm_r"]), int(row["pwm_r"])
            except (TypeError, ValueError, KeyError):
                continue


def load(paths, setpoint):
    """
    Samples where the wheel is stopped are dropped: the sketch is left running
    between tests, so every file has stretches at 0 RPM that would drag the
    steady-state mean down and silently corrupt the result.
    """
    data = defaultdict(lambda: defaultdict(list))
    for path in paths:
        head = open(path, errors="replace").readline()
        reader = read_csv_log if head.startswith("t_ms,") else read_table_capture
        n = 0
        for kp, wheel, rpm, pwm in reader(path, setpoint):
            if rpm > 1.0:
                data[kp][wheel].append((rpm, pwm))
                n += 1
        print(f"  {path}: {n} moving samples "
              f"({'CSV log' if reader is read_csv_log else 'table capture'})")
    return data


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+", help="Activity 3 captures")
    ap.add_argument("--setpoint", type=float, default=150.0)
    ap.add_argument("--out", default="kp_error.png")
    args = ap.parse_args()

    print("reading:")
    data = load(args.files, args.setpoint)
    if not data:
        sys.exit("no usable samples found")

    sp = args.setpoint
    print(f"\nSteady-state response at setpoint {sp:.0f} RPM\n")
    print(f"{'Kp':>6} {'wheel':>6} {'n':>5} {'act rpm':>9} {'sd':>6} "
          f"{'err rpm':>9} {'err %':>7} {'pwm':>6} {'Kp*e':>7} {'predicted':>10}")
    print("-" * 84)

    rows = []
    for kp in sorted(data):
        pred = predicted_error(kp, sp)
        for wheel in ("LEFT", "RIGHT"):
            s = data[kp].get(wheel, [])
            if len(s) < 5:
                continue
            rpm = [x[0] for x in s]
            pwm = [x[1] for x in s]
            mean = st.mean(rpm)
            err = sp - mean
            print(f"{kp:>6.2f} {wheel:>6} {len(s):>5} {mean:>9.1f} "
                  f"{st.pstdev(rpm):>6.1f} {err:>9.1f} {100*err/sp:>7.1f} "
                  f"{st.mean(pwm):>6.1f} {kp*err:>7.1f} {pred:>10.1f}")
            rows.append((kp, wheel, err))

    fig, ax = plt.subplots(figsize=(8.5, 5.5), dpi=160)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)

    kps = sorted(data)
    smooth = np.linspace(min(kps) * 0.8, max(kps) * 1.1, 200)
    ax.plot(smooth, [predicted_error(k, sp) for k in smooth],
            color=THEORY_COLOR, linewidth=2, linestyle="--",
            label="Predicted from Activity 1 model")

    for wheel in ("LEFT", "RIGHT"):
        pts = [(k, e) for k, w, e in rows if w == wheel]
        if pts:
            ax.plot([p[0] for p in pts], [p[1] for p in pts],
                    marker="o", markersize=6, linewidth=2,
                    markeredgecolor=SURFACE, markeredgewidth=1.2,
                    color=WHEEL_COLOR[wheel], label=f"{wheel.title()} measured")

    ax.set_xlabel("Proportional gain Kp", color=TEXT_SECONDARY)
    ax.set_ylabel(f"Steady-state error (RPM) at {sp:.0f} RPM setpoint",
                  color=TEXT_SECONDARY)
    ax.set_title("P control: steady-state error falls with Kp but never reaches zero",
                 color=TEXT_PRIMARY, fontsize=12, pad=12, loc="left")
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    ax.set_ylim(bottom=0)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=TEXT_SECONDARY)
    ax.legend(frameon=False, labelcolor=TEXT_SECONDARY, fontsize=9)

    fig.tight_layout()
    fig.savefig(args.out, facecolor=SURFACE)
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
