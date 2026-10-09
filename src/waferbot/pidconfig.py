"""Small, explicit configuration for physical PID edge following."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, fields
from pathlib import Path


@dataclass(frozen=True)
class PIDConfig:
    enabled: bool = False
    rate_hz: float = 100.0
    ir_polling_hz: float = 300.0
    max_sample_age_s: float = 0.25
    base_pwm: int = 4
    max_pwm: int = 5
    kp: float = 0.307692
    ki: float = 0.0
    kd: float = 0.009231
    integral_limit_pwm: float = 0.0
    max_correction_pwm: float = 4.0
    max_slew_pwm_per_s: float = 20.0
    velocity_filter_tau_s: float = 0.05
    sensor_positions_mm: tuple[float, float, float, float] = (-31.25, -3.25, 3.25, 31.25)
    reference_mm: float = 0.0
    edge_loss_samples: int = 3
    acquisition_samples: int = 3
    acquisition_timeout_s: float = 3.0
    min_curve_speed_factor: float = 0.45
    curve_slowdown_start: float = 0.2

    @classmethod
    def load_json(cls, path: str | Path) -> "PIDConfig":
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        allowed = {field.name for field in fields(cls)}
        extra = set(data) - allowed - {"_comment"}
        if extra:
            raise ValueError(f"unknown PID settings: {', '.join(sorted(extra))}")
        values = {key: value for key, value in data.items() if key in allowed}
        if "sensor_positions_mm" in values:
            values["sensor_positions_mm"] = tuple(values["sensor_positions_mm"])
        return cls(**values).validate()

    def validate(self) -> "PIDConfig":
        if type(self.enabled) is not bool:
            raise ValueError("enabled must be a boolean")
        for name in ("base_pwm", "max_pwm", "edge_loss_samples", "acquisition_samples"):
            value = getattr(self, name)
            if type(value) is not int or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        if self.base_pwm > self.max_pwm or self.max_pwm > 5:
            raise ValueError("base_pwm must be at most max_pwm, and max_pwm at most 5")
        for name in ("rate_hz", "ir_polling_hz", "max_sample_age_s", "max_slew_pwm_per_s", "velocity_filter_tau_s", "acquisition_timeout_s"):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if not 1 <= self.ir_polling_hz <= 1000 or self.ir_polling_hz < self.rate_hz:
            raise ValueError("ir_polling_hz must be 1..1000 and at least rate_hz")
        for name in ("kp", "ki", "kd", "integral_limit_pwm", "max_correction_pwm"):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.ki > 0 and self.integral_limit_pwm <= 0:
            raise ValueError("positive ki requires integral_limit_pwm")
        if len(self.sensor_positions_mm) != 4 or any(
            type(p) not in (int, float) or not math.isfinite(p) for p in self.sensor_positions_mm
        ) or any(b <= a for a, b in zip(self.sensor_positions_mm, self.sensor_positions_mm[1:])):
            raise ValueError("sensor_positions_mm must be four increasing finite numbers")
        if type(self.reference_mm) not in (int, float) or not math.isfinite(self.reference_mm):
            raise ValueError("reference_mm must be finite")
        for name in ("min_curve_speed_factor", "curve_slowdown_start"):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= 1:
                raise ValueError(f"{name} must be between 0 and 1")
        return self
