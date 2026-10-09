"""Geometry-aware, quantized edge position and velocity estimates.

Positive positions point right as viewed from behind the robot. ``edge_mm``
is a conservative control representative of the selected transition interval;
robot displacement relative to that representative is its negative. Binary IR
bits locate intervals, not exact positions. For an outer interval, use its
inner sensor rather than its distant midpoint so the first off-centre pattern
has the near-centre sensor coordinate (3.25 mm in layout A).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from .edge import EdgeState

DEFAULT_SENSOR_POSITIONS_MM = (-31.25, -3.25, 3.25, 31.25)


@dataclass(frozen=True)
class PositionEstimate:
    selected_edge: EdgeState
    edge_mm: float | None
    robot_mm: float | None
    lower_bound_mm: float | None
    upper_bound_mm: float | None
    valid: bool
    confidence: float
    timestamp_ns: int
    reason: str


def estimate_position(
    normalized: Sequence[int], selected_edge: EdgeState,
    timestamp_ns: int, positions_mm: Sequence[float] = DEFAULT_SENSOR_POSITIONS_MM,
) -> PositionEstimate:
    if selected_edge not in (EdgeState.BLACK_LEFT, EdgeState.BLACK_RIGHT):
        raise ValueError("selected_edge must be BLACK_LEFT or BLACK_RIGHT")
    if len(normalized) != 4 or any(type(bit) is not int or bit not in (0, 1) for bit in normalized):
        raise ValueError("normalized must contain four binary integer channels")
    if len(positions_mm) != 4 or any(not math.isfinite(float(p)) for p in positions_mm):
        raise ValueError("positions_mm must contain four finite positions")
    positions = tuple(float(p) for p in positions_mm)
    if any(right <= left for left, right in zip(positions, positions[1:])):
        raise ValueError("sensor positions must increase from left to right")
    if timestamp_ns < 0:
        raise ValueError("timestamp_ns must be non-negative")
    want = (1, 0) if selected_edge is EdgeState.BLACK_LEFT else (0, 1)
    candidates = [
        index for index in range(3)
        if (normalized[index], normalized[index + 1]) == want
    ]
    if len(candidates) != 1:
        reason = "no selected edge in view" if not candidates else "multiple selected edges"
        return PositionEstimate(selected_edge, None, None, None, None, False, 0.0, timestamp_ns, reason)
    index = candidates[0]
    lower, upper = positions[index:index + 2]
    # The centre transition has a useful symmetric midpoint. An outer
    # transition only proves the edge lies somewhere across a much wider gap;
    # its midpoint (17.25 mm in layout A) would overstate the displacement at
    # the first 1100 -> 1000 or 1100 -> 1110 observation. Use the endpoint
    # nearest the centre as the *control representative* and retain the full
    # physical interval in lower_bound_mm/upper_bound_mm.
    representative = (
        upper if index == 0 else lower if index == 2 else (lower + upper) / 2
    )
    # Seeing both sides of a finite band is useful but the binary transition
    # remains only an interval. A single isolated black channel is less robust.
    opposite = (0, 1) if want == (1, 0) else (1, 0)
    has_opposite = any(
        (normalized[j], normalized[j + 1]) == opposite for j in range(3)
    )
    confidence = 0.7 if has_opposite else 1.0
    return PositionEstimate(
        selected_edge, representative, -representative, lower, upper,
        True, confidence, timestamp_ns,
        "selected transition interval; nearest-centre representative"
        if index != 1 else "selected central transition interval",
    )


@dataclass(frozen=True)
class VelocityEstimate:
    robot_mm_s: float | None
    valid: bool
    quality: float
    timestamp_ns: int
    reason: str


class EdgeVelocityEstimator:
    """Bounded finite difference between distinct bins, with time-based LPF."""

    def __init__(self, *, tau_s: float = 0.05, min_transition_s: float = 0.01,
                 max_abs_mm_s: float = 250.0) -> None:
        if tau_s <= 0 or min_transition_s <= 0 or max_abs_mm_s <= 0:
            raise ValueError("velocity estimator parameters must be positive")
        self.tau_s = tau_s
        self.min_transition_s = min_transition_s
        self.max_abs_mm_s = max_abs_mm_s
        self.reset()

    def reset(self) -> None:
        self._last_distinct: PositionEstimate | None = None
        self._velocity = 0.0
        self._last_timestamp_ns: int | None = None

    def update(self, position: PositionEstimate) -> VelocityEstimate:
        if not position.valid:
            self.reset()
            return VelocityEstimate(None, False, 0.0, position.timestamp_ns, "invalid position")
        previous = self._last_distinct
        if self._last_timestamp_ns is not None and position.timestamp_ns <= self._last_timestamp_ns:
            return VelocityEstimate(None, False, 0.0, position.timestamp_ns, "non-increasing timestamp")
        self._last_timestamp_ns = position.timestamp_ns
        if previous is None:
            self._last_distinct = position
            return VelocityEstimate(0.0, True, 0.2, position.timestamp_ns, "initial bin")
        if position.robot_mm == previous.robot_mm:
            return VelocityEstimate(self._velocity, True, 0.3, position.timestamp_ns, "unchanged quantized bin")
        dt = (position.timestamp_ns - previous.timestamp_ns) / 1e9
        if dt < self.min_transition_s:
            return VelocityEstimate(self._velocity, True, 0.1, position.timestamp_ns, "transition too soon")
        raw = (position.robot_mm - previous.robot_mm) / dt
        bounded = max(-self.max_abs_mm_s, min(self.max_abs_mm_s, raw))
        alpha = math.exp(-dt / self.tau_s)
        self._velocity = alpha * self._velocity + (1 - alpha) * bounded
        self._last_distinct = position
        quality = min(position.confidence, 0.6) if abs(raw) <= self.max_abs_mm_s else 0.2
        return VelocityEstimate(self._velocity, True, quality, position.timestamp_ns, "transition estimate")
