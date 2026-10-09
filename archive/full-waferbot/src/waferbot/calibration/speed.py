"""Measurable speed calibration (PWM counts to metres per second).

Nothing in the navigation stack invents a velocity. To use ``speed_limit_mps``
meaningfully, measure it: drive a fixed distance, time it, and compute
m/s per count. This module does the arithmetic and keeps a conservative value
that the operator can paste into the navigation configuration.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Iterable


@dataclass(frozen=True)
class SpeedSample:
    counts: float
    distance_m: float
    duration_s: float

    @property
    def speed_mps(self) -> float:
        return self.distance_m / self.duration_s if self.duration_s else 0.0

    @property
    def mps_per_count(self) -> float:
        return self.speed_mps / self.counts if self.counts else 0.0

    def as_dict(self) -> dict[str, Any]:
        return {
            "counts": self.counts,
            "distance_m": self.distance_m,
            "duration_s": self.duration_s,
            "speed_mps": self.speed_mps,
            "mps_per_count": self.mps_per_count,
        }


@dataclass
class SpeedCalibrationReport:
    samples: list[SpeedSample] = field(default_factory=list)
    conservatism: float = 0.8
    notes: list[str] = field(default_factory=list)

    @property
    def recommended_counts_to_mps(self) -> float | None:
        return compute_counts_to_mps(self.samples, conservatism=self.conservatism)

    @property
    def fastest_speed_mps(self) -> float:
        return max((sample.speed_mps for sample in self.samples), default=0.0)

    def as_dict(self) -> dict[str, Any]:
        return {
            "samples": [sample.as_dict() for sample in self.samples],
            "recommended_counts_to_mps": self.recommended_counts_to_mps,
            "fastest_speed_mps": self.fastest_speed_mps,
            "conservatism": self.conservatism,
            "notes": self.notes,
            "instructions": speed_calibration_instructions(),
        }


def compute_counts_to_mps(
    samples: Iterable[SpeedSample], *, conservatism: float = 0.8
) -> float | None:
    """Conservative *upper bound* on m/s per PWM count from measured runs.

    Speed limits are enforced as ``counts = limit_mps / factor``, so the factor
    must over-estimate the real speed for the resulting command to stay under
    the requested limit. We therefore take the **largest** measured ratio and
    divide by ``conservatism`` (a number <= 1), which inflates it further.

    Uncertainty: these runs measure straight-line travel on one surface, one
    battery charge, and one load. Strafe/diagonal moves, battery sag, and extra
    payload can all be faster or slower; re-measure before trusting a limit.
    """
    if not 0 < conservatism <= 1:
        raise ValueError("conservatism must be in (0, 1]")
    ratios = []
    for sample in samples:
        for name, value in (
            ("counts", sample.counts),
            ("distance_m", sample.distance_m),
            ("duration_s", sample.duration_s),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(f"speed sample {name} must be a finite number")
        if sample.counts <= 0 or sample.distance_m <= 0 or sample.duration_s <= 0:
            raise ValueError(
                "speed samples need positive counts, distance, and duration"
            )
        ratios.append(sample.mps_per_count)
    if not ratios:
        return None
    return max(ratios) / conservatism


def speed_calibration_instructions() -> str:
    return (
        "Speed calibration (do this on the real floor, wheels down):\n"
        "  1. Mark a straight lane of at least 1.0 m on the tape and measure it.\n"
        "  2. For each PWM count you care about (for example 30, 50, 70), run the\n"
        "     robot along the lane at that fixed count (bench test, mock-safe\n"
        "     equivalent: the same command in mock mode) and time how long it\n"
        "     takes to cover the measured distance.\n"
        "  3. Record counts, distance, and duration with\n"
        "     `waferbot calibrate speed --counts C --distance-m D --duration-s T`.\n"
        "  4. Use the reported `recommended_counts_to_mps`, which is the FASTEST\n"
        "     measured m/s-per-count inflated by the conservatism factor, in\n"
        "     nav.counts_to_mps. Inflating it makes the derived PWM caps slower,\n"
        "     never faster.\n"
        "  5. Re-measure for strafing/diagonal moves, a different payload, and a\n"
        "     different battery charge; a single straight-line run is not a\n"
        "     general calibration.\n"
        "  Never enter a guessed factor: an uncalibrated value stays null and the\n"
        "  stack keeps reporting PWM counts instead of metres per second."
    )


__all__ = [
    "SpeedCalibrationReport",
    "SpeedSample",
    "compute_counts_to_mps",
    "speed_calibration_instructions",
]
