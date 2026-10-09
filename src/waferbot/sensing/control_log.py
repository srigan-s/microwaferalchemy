"""Per-control-update CSV diagnostics, separate from high-rate IR telemetry."""

from __future__ import annotations

import csv
from pathlib import Path

FIELDS = (
    "timestamp_ns", "elapsed_s", "control_dt_s", "raw_ir", "normalized_ir",
    "edge_state", "selected_edge", "edge_position_mm", "robot_position_mm",
    "position_lower_bound_mm", "position_upper_bound_mm", "velocity_mm_s",
    "measurement_valid", "measurement_age_ms", "reference_mm", "error_mm",
    "kp_pwm_per_mm", "ki_pwm_per_mm_s", "kd_pwm_s_per_mm", "p_term",
    "i_term", "d_term", "raw_correction", "limited_correction",
    "front_left_pwm", "rear_left_pwm", "front_right_pwm", "rear_right_pwm",
    "output_saturated", "safety_state", "control_state", "polling_rate_hz",
    "control_rate_hz", "missed_poll_deadlines", "skipped_ir_samples",
    "missed_control_deadlines",
    "poll_latency_ms", "pid_compute_ms", "motor_write_ms", "feedback_to_actuation_ms",
)


class ControlCSV:
    def __init__(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._handle, fieldnames=FIELDS)
        self._writer.writeheader()

    def write(self, **data) -> None:
        self._writer.writerow({field: data.get(field) for field in FIELDS})
        self._handle.flush()

    def close(self) -> None:
        self._handle.close()
