#!/usr/bin/env python3
"""Offline quantized-IR PID comparison with an explicitly idealized yaw plant.

Uses the production position, velocity, PID, and wheel mixer. This is a
signal-level tuning aid, not fitted hardware dynamics or a 90-degree guarantee.
Requires matplotlib on the development computer; it is not needed on the Pi.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

from waferbot.sensing.edge import EdgeState
from waferbot.sensing.follower import wheel_command
from waferbot.sensing.pid import PIDController, PIDGains
from waferbot.sensing.position import EdgeVelocityEstimator, estimate_position

LAYOUTS = {
    "a": (-31.25, -3.25, 3.25, 31.25),
    "b": (-31.25, -17.25, 17.25, 31.25),
}


@dataclass(frozen=True)
class Sample:
    time_s: float
    error_mm: float | None
    correction_pwm: float
    wheels: tuple[int, int, int, int]
    pattern: str


def simulate(*, layout: str, kp: float, ki: float, kd: float,
             tau_s: float, poll_hz: int, control_hz: int,
             duration_s: float = 5.0, tape_width_mm: float = 30.0,
             tolerance_mm: float = 1.0, mode: str = "lateral",
             lateral_weight: float = 0.5) -> tuple[list[Sample], float | None]:
    if poll_hz < control_hz or min(poll_hz, control_hz) <= 0:
        raise ValueError("poll_hz must be at least control_hz > 0")
    positions = LAYOUTS[layout]
    pid = PIDController(PIDGains(kp=kp, ki=ki, kd=kd,
                                integral_limit_pwm=2 if ki else 0,
                                max_correction_pwm=5, max_slew_pwm_per_s=60,
                                initial_dt_s=1 / control_hz))
    velocity = EdgeVelocityEstimator(tau_s=tau_s)
    # Synthetic pose. The selected black-left edge starts slightly left of
    # the centre gap, creating a visible quantized initial error.
    x, y, heading = 0.0, -0.025, 0.0
    wheels = (0, 0, 0, 0)
    output: list[Sample] = []
    next_control = 0.0
    dt = 1 / poll_hz
    steps = round(duration_s * poll_hz)
    for tick in range(steps + 1):
        t = tick * dt
        # Finite-width tape and 6 mm binary sensor footprint. S1 is leftmost;
        # controller positions increase toward robot right.
        bits = []
        for position_mm in positions:
            sensor_y_mm = 1000 * (y + 0.10 * math.sin(heading)) - position_mm
            low, high = sensor_y_mm - 3, sensor_y_mm + 3
            overlap = max(0.0, min(high, tape_width_mm / 2) -
                          max(low, -tape_width_mm / 2))
            bits.append(int(overlap >= 3))
        ns = round(t * 1e9)
        measured = estimate_position(bits, EdgeState.BLACK_LEFT, ns, positions)
        speed = velocity.update(measured)
        if t + 1e-12 >= next_control:
            next_control += 1 / control_hz
            action = pid.update(reference_mm=0, measurement=measured,
                                velocity=speed, timestamp_ns=ns)
            if action.measurement_valid:
                wheels = wheel_command(base_speed=3, correction=action.output_pwm,
                                       mode=mode, lateral_weight=lateral_weight,
                                       limit=5)
                if 3 + abs(action.output_pwm) > 5:
                    pid.note_motor_saturation()
            else:
                wheels = (0, 0, 0, 0)
            output.append(Sample(t, action.error_mm, action.output_pwm,
                                 wheels, "".join(map(str, bits))))
        # Same sign convention as TapeModelTransport, with idealized kinematic
        # coefficients; no static friction, slip, or measured motor lag.
        forward = sum(wheels) / 4
        lateral = (-wheels[0] + wheels[1] + wheels[2] - wheels[3]) / 4
        yaw = (-wheels[0] - wheels[1] + wheels[2] + wheels[3]) / 4
        heading += yaw * 0.010 * dt
        x += (forward * math.cos(heading) - lateral * math.sin(heading)) * 0.002 * dt
        y += (forward * math.sin(heading) + lateral * math.cos(heading)) * 0.002 * dt
    # Settling only exists if the quantized estimate remains inside an explicit
    # tolerance through the end; a final lost edge is never called settled.
    settled: float | None = None
    for index, sample in enumerate(output):
        if all(later.error_mm is not None and abs(later.error_mm) <= tolerance_mm
               for later in output[index:]):
            settled = sample.time_s
            break
    return output, settled


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--layout", choices=LAYOUTS, default="a")
    parser.add_argument("--kp", type=float, default=0.7)
    parser.add_argument("--ki", type=float, default=0.0)
    parser.add_argument("--kd", type=float, default=0.015)
    parser.add_argument("--tau-s", type=float, default=0.05)
    parser.add_argument("--poll-hz", type=int, default=300)
    parser.add_argument("--control-hz", type=int, default=100)
    parser.add_argument("--mode", choices=("lateral", "differential", "blended"),
                        default="lateral")
    parser.add_argument("--lateral-weight", type=float, default=0.5)
    parser.add_argument("--tolerance-mm", type=float, default=1)
    parser.add_argument("--output-prefix", type=Path, default=Path("pid-comparison"))
    args = parser.parse_args()
    samples, settled = simulate(layout=args.layout, kp=args.kp, ki=args.ki,
                                kd=args.kd, tau_s=args.tau_s, poll_hz=args.poll_hz,
                                control_hz=args.control_hz, mode=args.mode,
                                lateral_weight=args.lateral_weight,
                                tolerance_mm=args.tolerance_mm)
    csv_path = args.output_prefix.with_suffix(".csv")
    png_path = args.output_prefix.with_suffix(".png")
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("time_s", "error_mm", "correction_pwm", "fl", "rl", "fr", "rr", "pattern"))
        for sample in samples:
            writer.writerow((sample.time_s, sample.error_mm, sample.correction_pwm,
                             *sample.wheels, sample.pattern))
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 7))
    t = [s.time_s for s in samples]
    axes[0].plot(t, [s.error_mm for s in samples])
    axes[0].axhline(args.tolerance_mm, color="gray", ls="--")
    axes[0].axhline(-args.tolerance_mm, color="gray", ls="--")
    axes[0].set_ylabel("Position error (mm)")
    axes[1].plot(t, [s.correction_pwm for s in samples])
    axes[1].set_ylabel("PID correction (PWM)")
    for index, label in enumerate(("FL", "RL", "FR", "RR")):
        axes[2].step(t, [s.wheels[index] for s in samples], label=label)
    axes[2].legend(ncol=4)
    axes[2].set_ylabel("Motor PWM")
    axes[2].set_xlabel("Time (s)")
    fig.tight_layout()
    fig.savefig(png_path, dpi=150)
    plt.close(fig)
    print(f"{csv_path} {png_path}; settling within ±{args.tolerance_mm:g} mm: "
          f"{settled if settled is not None else 'not observed'}; idealized plant")


if __name__ == "__main__":
    main()
