"""Single discrete-time PID law for the Waferbot edge follower.

Gains use PWM/mm, PWM/(mm*s), and PWM*s/mm. Derivative is applied to the
filtered measured robot velocity; there is no second derivative path.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .position import PositionEstimate, VelocityEstimate


@dataclass(frozen=True)
class PIDGains:
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    integral_limit_pwm: float = 0.0
    max_correction_pwm: float = 5.0
    max_slew_pwm_per_s: float = 100.0
    initial_dt_s: float = 0.01

    def validate(self) -> "PIDGains":
        for name, value in vars(self).items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.ki > 0 and self.integral_limit_pwm <= 0:
            raise ValueError("positive Ki needs a positive integral limit")
        if self.max_slew_pwm_per_s <= 0:
            raise ValueError("max_slew_pwm_per_s must be positive")
        if self.initial_dt_s <= 0:
            raise ValueError("initial_dt_s must be positive")
        return self


@dataclass(frozen=True)
class PIDOutput:
    error_mm: float | None
    position_mm: float | None
    velocity_mm_s: float | None
    p_term: float
    i_term: float
    d_term: float
    output_raw: float
    output_limited: float
    output_pwm: float
    saturated: bool
    measurement_valid: bool
    dt_s: float | None


class PIDController:
    def __init__(self, gains: PIDGains) -> None:
        self.gains = gains.validate()
        self.reset()

    def reset(self) -> None:
        self._last_ns: int | None = None
        self._last_output = 0.0
        self._integral = 0.0
        self._last_integral_step = 0.0

    @property
    def integral_pwm(self) -> float:
        return self._integral

    def note_motor_saturation(self) -> None:
        """Roll back the most recent integral step if the mixer hit a wheel cap."""
        self._integral -= self._last_integral_step
        self._last_integral_step = 0.0

    def update(
        self, *, reference_mm: float, measurement: PositionEstimate,
        velocity: VelocityEstimate, timestamp_ns: int,
    ) -> PIDOutput:
        valid = measurement.valid and velocity.valid and measurement.robot_mm is not None
        if not valid or timestamp_ns < measurement.timestamp_ns:
            self.reset()
            return PIDOutput(None, measurement.robot_mm, velocity.robot_mm_s,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False, None)
        if not math.isfinite(reference_mm):
            raise ValueError("reference_mm must be finite")
        if self._last_ns is not None and timestamp_ns <= self._last_ns:
            self.reset()
            return PIDOutput(None, measurement.robot_mm, velocity.robot_mm_s,
                             0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, False, None)
        dt = None if self._last_ns is None else (timestamp_ns - self._last_ns) / 1e9
        self._last_ns = timestamp_ns
        error = reference_mm - measurement.robot_mm
        p_term = self.gains.kp * error
        d_term = -self.gains.kd * (velocity.robot_mm_s or 0.0)
        previous_i = self._integral
        candidate_i = previous_i
        if dt is not None and self.gains.ki:
            candidate_i = max(
                -self.gains.integral_limit_pwm,
                min(self.gains.integral_limit_pwm, previous_i + self.gains.ki * error * dt),
            )
        raw_candidate = p_term + candidate_i + d_term
        limit = self.gains.max_correction_pwm
        # Conditional integration prevents further windup in the saturated
        # direction, but permits unwinding if error reverses.
        if abs(raw_candidate) > limit and error * raw_candidate > 0:
            candidate_i = previous_i
        self._integral = candidate_i
        self._last_integral_step = candidate_i - previous_i
        raw = p_term + self._integral + d_term
        limited = max(-limit, min(limit, raw))
        delta = self.gains.max_slew_pwm_per_s * (self.gains.initial_dt_s if dt is None else dt)
        output = max(self._last_output - delta, min(self._last_output + delta, limited))
        self._last_output = output
        return PIDOutput(error, measurement.robot_mm, velocity.robot_mm_s,
                         p_term, self._integral, d_term, raw, limited, output,
                         abs(raw) > limit or output != limited, True, dt)
