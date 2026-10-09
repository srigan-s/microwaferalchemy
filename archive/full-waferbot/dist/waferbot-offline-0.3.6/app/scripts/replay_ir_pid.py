#!/usr/bin/env python3
"""Replay recorded IR experiment CSV through the production estimator and PID.

No hardware access or motor writes. Input must contain real read_end_ns and
black_s1..black_s4 columns from ir-strafe or edge-angle-sweep.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

from waferbot.navconfig import NavConfig
from waferbot.sensing.edge import EdgeState
from waferbot.sensing.pid import PIDController, PIDGains
from waferbot.sensing.position import EdgeVelocityEstimator, estimate_position


FIELDS = ("read_end_ns", "normalized_ir", "selected_edge", "edge_mm",
          "robot_mm", "lower_mm", "upper_mm", "velocity_mm_s", "valid",
          "reason", "error_mm", "p_pwm", "i_pwm", "d_pwm", "output_pwm")


def replay(input_path: Path, output_path: Path, *, edge: EdgeState,
           nav: NavConfig) -> int:
    follow = nav.follow
    gap = follow.sensor_positions_mm[2] - follow.sensor_positions_mm[1]
    pid = PIDController(PIDGains(
        kp=follow.kp_pwm_per_mm if follow.kp_pwm_per_mm is not None else follow.kp / gap,
        ki=follow.ki_pwm_per_mm_s,
        kd=follow.kd_pwm_s_per_mm if follow.kd_pwm_s_per_mm is not None else follow.kd / gap,
        integral_limit_pwm=follow.integral_limit_pwm,
        max_correction_pwm=follow.max_correction,
        max_slew_pwm_per_s=(follow.max_correction_slew_pwm_per_s
                            if follow.max_correction_slew_pwm_per_s is not None
                            else follow.max_correction_delta * 20),
        initial_dt_s=1 / follow.rate_hz,
    ))
    velocity = EdgeVelocityEstimator(tau_s=follow.velocity_filter_tau_s)
    period_ns = round(1e9 / follow.rate_hz)
    previous_ns: int | None = None
    next_control_ns: int | None = None
    count = 0
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with input_path.open(newline="", encoding="utf-8") as source, \
            output_path.open("w", newline="", encoding="utf-8") as destination:
        reader = csv.DictReader(source)
        required = {"read_end_ns", "black_s1", "black_s2", "black_s3", "black_s4"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"input requires columns {', '.join(sorted(required))}")
        writer = csv.DictWriter(destination, fieldnames=FIELDS)
        writer.writeheader()
        for row in reader:
            ns = int(row["read_end_ns"])
            if previous_ns is not None and ns <= previous_ns:
                raise ValueError("read_end_ns must strictly increase; timestamps are never invented")
            previous_ns = ns
            bits = tuple(int(row[f"black_s{i}"]) for i in range(1, 5))
            measured = estimate_position(bits, edge, ns, follow.sensor_positions_mm)
            speed = velocity.update(measured)
            if next_control_ns is None:
                next_control_ns = ns
            if ns < next_control_ns:
                continue
            next_control_ns = ns + period_ns
            action = pid.update(reference_mm=follow.reference_mm,
                                measurement=measured, velocity=speed, timestamp_ns=ns)
            writer.writerow(dict(zip(FIELDS, (
                ns, "".join(map(str, bits)), edge.value, measured.edge_mm,
                measured.robot_mm, measured.lower_bound_mm, measured.upper_bound_mm,
                speed.robot_mm_s, measured.valid, measured.reason, action.error_mm,
                action.p_term, action.i_term, action.d_term, action.output_pwm,
            ))))
            count += 1
    return count


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--edge", choices=("black-left", "black-right"), required=True)
    parser.add_argument("--nav-config", type=Path, default=Path("config/nav.windy-first-run.json"))
    args = parser.parse_args()
    count = replay(args.input, args.output, edge=EdgeState.from_name(args.edge),
                   nav=NavConfig.load_json(args.nav_config))
    print(f"Replayed {count} timestamped control updates to {args.output}; no motors commanded")


if __name__ == "__main__":
    main()
