#!/usr/bin/env python3
"""Plot one Waferbot controller CSV; writes PNGs to the current directory."""

import csv
import math
import sys

import matplotlib

matplotlib.use("Agg")  # Save plots without a desktop display (for example, over SSH).
import matplotlib.pyplot as plt


def number(row, field):
    """Return a finite number, or NaN to leave a gap in the plot."""
    try:
        value = float(row.get(field, ""))
        return value if math.isfinite(value) else math.nan
    except (TypeError, ValueError):
        return math.nan


def save_plot(filename, title, ylabel, times, series, *, steps=False):
    fig, ax = plt.subplots(figsize=(10, 4))
    for label, values in series.items():
        if steps:
            ax.step(times, values, where="post", label=label)
        else:
            ax.plot(times, values, label=label)
    ax.set(title=title, xlabel="Time (s)", ylabel=ylabel)
    if steps:
        ax.set_yticks([0, 1])
        ax.set_ylim(-0.2, 1.2)
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(filename, dpi=150)
    plt.close(fig)


def main(csv_file):
    rows = []
    skipped = 0
    with open(csv_file, newline="", encoding="utf-8-sig") as handle:
        for row in csv.DictReader(handle):
            elapsed = number(row, "elapsed_s")
            if math.isnan(elapsed):
                skipped += 1
                continue
            rows.append((elapsed, row))

    if not rows:
        raise SystemExit("No rows with a valid elapsed_s value were found.")

    times = [elapsed for elapsed, _ in rows]
    fields = {
        "robot_position_mm": "Robot position",
        "reference_mm": "Reference",
        "raw_correction": "Raw correction",
        "limited_correction": "Limited correction",
        "p_term": "P",
        "i_term": "I",
        "d_term": "D",
        "front_left_pwm": "Front left",
        "rear_left_pwm": "Rear left",
        "front_right_pwm": "Front right",
        "rear_right_pwm": "Rear right",
    }
    values = {
        field: [number(row, field) for _, row in rows]
        for field in fields
    }

    # An invalid bit pattern creates a gap. Valid line-loss patterns such as
    # 0000 are plotted normally, even if the controller stopped on that row.
    ir = [str(row.get("normalized_ir") or "").strip() for _, row in rows]
    ir_series = {
        f"S{index + 1}": [
            int(bits[index]) if len(bits) == 4 and set(bits) <= {"0", "1"}
            else math.nan
            for bits in ir
        ]
        for index in range(4)
    }
    save_plot("ir_vs_time.png", "IR sensors vs time", "Black (1) / white (0)",
              times, ir_series, steps=True)

    plots = (
        ("position_vs_time.png", "Position vs time", "Position (mm)",
         ("robot_position_mm", "reference_mm")),
        ("pid_correction_vs_time.png", "PID correction vs time", "Correction (PWM)",
         ("raw_correction", "limited_correction")),
        ("pid_terms_vs_time.png", "PID terms vs time", "PWM contribution",
         ("p_term", "i_term", "d_term")),
        ("motor_pwm_vs_time.png", "Motor PWM vs time", "Signed PWM count",
         ("front_left_pwm", "rear_left_pwm", "front_right_pwm", "rear_right_pwm")),
    )
    for filename, title, ylabel, columns in plots:
        save_plot(filename, title, ylabel, times,
                  {fields[column]: values[column] for column in columns})

    print(f"Saved 5 PNGs in the current directory ({skipped} rows skipped).")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("Usage: python3 plot_pid.py <csv_file>")
    main(sys.argv[1])
