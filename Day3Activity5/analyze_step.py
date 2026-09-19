#!/usr/bin/env python3
"""
analyze_step.py - plot the step response logged by Day3Activity5.ino and
extract rise time, settling time, overshoot and steady-state error.

Usage:
    python3 analyze_step.py step_log.csv
    python3 analyze_step.py step_log.csv --out step_response.png --wheel right

Definitions used (standard control-systems conventions):
    rise time      10% -> 90% of the commanded step change
    settling time  time to enter and stay inside a +/-2% band around the
                   final value, measured from the step instant
    overshoot      100 * (peak - final) / step_change, 0 if it never exceeds
    ss error       setpoint - mean(actual over the last 25% of the step)

Requires: matplotlib, numpy   ->   pip install matplotlib numpy
"""

import argparse
import csv
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

SETPOINT_COLOR = "#52514e"
WHEEL_COLOR = {"left": "#2a78d6", "right": "#eb6834"}
SURFACE = "#fcfcfb"
TEXT_PRIMARY = "#0b0b0b"
TEXT_SECONDARY = "#52514e"
GRID = "#e3e2de"

SETTLE_BAND = 0.02   # +/-2%
RISE_LO, RISE_HI = 0.10, 0.90


def load(path):
    """Parse the sketch's CSV, tolerating comment lines and serial glitches."""
    rows = []
    with open(path, newline="", errors="replace") as fh:
        reader = csv.DictReader(
            (ln for ln in fh if ln.strip() and not ln.startswith("#"))
        )
        for r in reader:
            try:
                rows.append({
                    "t": float(r["t_ms"]) / 1000.0,
                    "step": int(r["step_idx"]),
                    "sp": float(r["setpoint_rpm"]),
                    "left": float(r["rpm_l"]),
                    "right": float(r["rpm_r"]),
                })
            except (TypeError, ValueError, KeyError):
                continue
    if not rows:
        sys.exit(f"no usable rows in {path}")
    return rows


def analyze_segment(t, y, start_value, setpoint):
    """
    Metrics for one step. t is relative to the step instant.

    Returns a dict; entries that cannot be determined from the captured window
    are None rather than a guessed number - a step that never settles inside
    the band has no settling time, and saying so is more useful than
    reporting the last sample.
    """
    step_change = setpoint - start_value
    out = {"rise": None, "settle": None, "overshoot": None, "sserr": None,
           "final": None}
    if len(t) < 3 or abs(step_change) < 1e-6:
        return out

    # Steady state from the tail of the segment, not the single last sample.
    tail = y[int(len(y) * 0.75):]
    final = float(np.mean(tail)) if len(tail) else float(y[-1])
    out["final"] = final
    out["sserr"] = setpoint - final

    lo = start_value + RISE_LO * step_change
    hi = start_value + RISE_HI * step_change

    def first_cross(level):
        if step_change > 0:
            idx = np.nonzero(y >= level)[0]
        else:
            idx = np.nonzero(y <= level)[0]
        return t[idx[0]] if idx.size else None

    t_lo, t_hi = first_cross(lo), first_cross(hi)
    if t_lo is not None and t_hi is not None and t_hi >= t_lo:
        out["rise"] = t_hi - t_lo

    # Overshoot is measured against the achieved final value, so a controller
    # with steady-state error is not credited with overshoot it did not have.
    peak = float(np.max(y)) if step_change > 0 else float(np.min(y))
    excess = (peak - final) if step_change > 0 else (final - peak)
    out["overshoot"] = max(0.0, 100.0 * excess / abs(step_change))

    # Band is 2% of the final value, but never smaller than 2% of the step
    # itself - otherwise a step back down to 0 RPM gets a band of nearly zero
    # width and nothing ever "settles".
    band = SETTLE_BAND * max(abs(final), abs(step_change))
    outside = np.nonzero(np.abs(y - final) > band)[0]
    if outside.size == 0:
        out["settle"] = 0.0
    elif outside[-1] + 1 < len(t):
        out["settle"] = float(t[outside[-1] + 1])
    # else: never settled inside the window -> stays None

    return out


def fmt(v, unit="", nd=3):
    return "—" if v is None else f"{v:.{nd}f}{unit}"


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", help="CSV captured from Day3Activity5")
    ap.add_argument("--out", default="step_response.png")
    ap.add_argument("--wheel", default="both",
                    choices=["left", "right", "both"])
    args = ap.parse_args()

    rows = load(args.csv)
    t = np.array([r["t"] for r in rows])
    sp = np.array([r["sp"] for r in rows])
    step = np.array([r["step"] for r in rows])
    data = {w: np.array([r[w] for r in rows]) for w in ("left", "right")}

    wheels = ["left", "right"] if args.wheel == "both" else [args.wheel]

    fig, ax = plt.subplots(figsize=(9.5, 5.5), dpi=160)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)

    ax.plot(t, sp, color=SETPOINT_COLOR, linewidth=2, linestyle="--",
            label="Setpoint")
    for w in wheels:
        ax.plot(t, data[w], color=WHEEL_COLOR[w], linewidth=2,
                label=f"{w.title()} wheel")

    print(f"\n{'step':>4} {'from':>8} {'to':>8} {'wheel':>6} {'rise':>9}"
          f" {'settle':>9} {'overshoot':>10} {'ss error':>10} {'final':>9}")
    print("-" * 80)

    prev_sp = 0.0
    for idx in sorted(set(step)):
        mask = step == idx
        if mask.sum() < 3:
            continue
        seg_t = t[mask] - t[mask][0]
        target = float(sp[mask][0])
        ax.axvline(t[mask][0], color=GRID, linewidth=1, zorder=0)

        for w in wheels:
            m = analyze_segment(seg_t, data[w][mask], prev_sp, target)
            print(f"{idx:>4} {prev_sp:>8.0f} {target:>8.0f} {w:>6}"
                  f" {fmt(m['rise'], ' s'):>9} {fmt(m['settle'], ' s'):>9}"
                  f" {fmt(m['overshoot'], ' %', 1):>10}"
                  f" {fmt(m['sserr'], '', 1):>10}"
                  f" {fmt(m['final'], '', 1):>9}")
        prev_sp = target

    ax.set_xlabel("Time (s)", color=TEXT_SECONDARY)
    ax.set_ylabel("Wheel speed (RPM)", color=TEXT_SECONDARY)
    ax.set_title("Wheel velocity controller step response",
                 color=TEXT_PRIMARY, fontsize=13, pad=12, loc="left")
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
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
