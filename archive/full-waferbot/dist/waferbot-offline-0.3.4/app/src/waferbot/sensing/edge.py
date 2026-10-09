"""Four-channel edge detection with freshness-aware debouncing.

Sensor contract (see ``HARDWARE_IMPLEMENTATION.md``): ``LineReading.normalized``
is ordered S1..S4 left to right with ``BLACK = 1`` and ``WHITE = 0``. The middle
pair S2/S3 classifies the edge:

| S2 | S3 | State |
| --- | --- | --- |
| BLACK | WHITE | ``BLACK_LEFT`` |
| WHITE | BLACK | ``BLACK_RIGHT`` |
| BLACK | BLACK | ``BOTH_BLACK`` (ambiguous) |
| WHITE | WHITE | ``BOTH_WHITE`` (ambiguous) |

Debouncing counts consecutive *fresh* readings with the same classification. A
stale reading (older than ``max_age_s``) resets the run, so ambiguous middle bits
never become a stable edge merely because time passed.
"""

from __future__ import annotations

import math
import time
from collections.abc import Sequence
from dataclasses import dataclass
from enum import Enum

from ..config import SensorConfig
from ..errors import SensorError
from ..hardware.sensors import LineReading, decode_line_byte


class EdgeState(str, Enum):
    """Edge classification of the middle sensor pair."""

    BLACK_LEFT = "BLACK_LEFT"
    BLACK_RIGHT = "BLACK_RIGHT"
    BOTH_BLACK = "BOTH_BLACK"
    BOTH_WHITE = "BOTH_WHITE"
    #: Only used for "no stable classification yet".
    UNKNOWN = "UNKNOWN"

    @property
    def is_ambiguous(self) -> bool:
        return self in (EdgeState.BOTH_BLACK, EdgeState.BOTH_WHITE)

    @property
    def is_edge(self) -> bool:
        return self in (EdgeState.BLACK_LEFT, EdgeState.BLACK_RIGHT)

    @property
    def opposite(self) -> "EdgeState":
        if self is EdgeState.BLACK_LEFT:
            return EdgeState.BLACK_RIGHT
        if self is EdgeState.BLACK_RIGHT:
            return EdgeState.BLACK_LEFT
        raise ValueError(f"{self.value} has no opposite edge")

    @classmethod
    def from_name(cls, name: str) -> "EdgeState":
        """Accept ``black-left``/``BLACK_LEFT`` style spellings."""
        key = str(name).strip().upper().replace("-", "_").replace(" ", "_")
        try:
            return cls(key)
        except ValueError as exc:
            raise ValueError(
                f"unknown edge {name!r}; expected BLACK_LEFT or BLACK_RIGHT"
            ) from exc


@dataclass(frozen=True)
class EdgeDetection:
    """One debounced observation."""

    raw: tuple[int, int, int, int]
    normalized: tuple[int, int, int, int]
    edge: EdgeState
    timestamp: float
    stable: bool
    stable_edge: EdgeState
    stable_count: int
    age_s: float
    outer_left: bool
    outer_right: bool
    junction: bool
    rejected: bool = False
    reject_reason: str | None = None

    @property
    def stale(self) -> bool:
        return not self.stable

    def as_dict(self) -> dict[str, object]:
        return {
            "raw": list(self.raw),
            "normalized": list(self.normalized),
            "edge": self.edge.value,
            "timestamp": self.timestamp,
            "stable": self.stable,
            "stable_edge": self.stable_edge.value,
            "stable_count": self.stable_count,
            "age_s": self.age_s,
            "outer_left": self.outer_left,
            "outer_right": self.outer_right,
            "junction": self.junction,
            "rejected": self.rejected,
            "reject_reason": self.reject_reason,
        }


def classify_middle(normalized: Sequence[int]) -> EdgeState:
    """Classify the middle pair of a normalised S1..S4 reading."""
    if len(normalized) != 4:
        raise SensorError(
            f"expected four normalised channels, got {list(normalized)!r}"
        )
    s2 = int(normalized[1])
    s3 = int(normalized[2])
    if s2 and not s3:
        return EdgeState.BLACK_LEFT
    if s3 and not s2:
        return EdgeState.BLACK_RIGHT
    if s2 and s3:
        return EdgeState.BOTH_BLACK
    return EdgeState.BOTH_WHITE


def _coerce_reading(
    sensor_values: LineReading | int | Sequence[int],
    config: SensorConfig | None,
    timestamp: float | None,
) -> LineReading:
    if isinstance(sensor_values, LineReading):
        return sensor_values
    if isinstance(sensor_values, bool):
        raise SensorError("sensor values must be a LineReading, byte, or 4 channels")
    if isinstance(sensor_values, int):
        return decode_line_byte(sensor_values, config, timestamp=timestamp)
    if isinstance(sensor_values, Sequence) and len(sensor_values) == 4:
        config = config or SensorConfig()
        raw = []
        normalized = []
        for value in sensor_values:
            if isinstance(value, bool) or not isinstance(value, int) or value not in (0, 1):
                raise SensorError(
                    f"raw channel values must be 0 or 1, got {value!r}"
                )
            raw.append(value)
            # Normalise to the project convention: BLACK = 1, WHITE = 0.
            normalized.append(1 - value if config.black_is_raw_zero else value)
        return LineReading(
            raw_byte=sum(
                bit << bit_index for bit, bit_index in zip(raw, config.bit_for_channel)
            ),
            raw=tuple(raw),  # type: ignore[arg-type]
            normalized=tuple(normalized),  # type: ignore[arg-type]
            timestamp=time.monotonic() if timestamp is None else timestamp,
        )
    raise SensorError(
        f"unsupported sensor values: {sensor_values!r}; pass a LineReading, a "
        "raw byte, or four raw channel bits"
    )


class EdgeDetector:
    """Debounces edge classifications using monotonic timestamps."""

    def __init__(
        self,
        *,
        stable_samples: int = 3,
        max_age_s: float = 0.25,
        junction_channels: int = 3,
        clock=time.monotonic,
    ) -> None:
        if stable_samples < 1:
            raise ValueError("stable_samples must be >= 1")
        if max_age_s <= 0:
            raise ValueError("max_age_s must be positive")
        self.stable_samples = int(stable_samples)
        self.max_age_s = float(max_age_s)
        self.junction_channels = int(junction_channels)
        self._clock = clock
        self._run_edge: EdgeState | None = None
        self._run_count = 0
        self._last: EdgeDetection | None = None
        self._last_timestamp: float | None = None
        self.last_rejection: str | None = None

    @property
    def last(self) -> EdgeDetection | None:
        return self._last

    def reset(self) -> None:
        self._run_edge = None
        self._run_count = 0
        self._last = None
        self._last_timestamp = None
        self.last_rejection = None

    def update(
        self, reading: LineReading, *, now: float | None = None
    ) -> EdgeDetection:
        moment = self._clock() if now is None else now
        self._validate_reading(reading)
        timestamp = float(reading.timestamp)
        age = moment - timestamp
        edge = classify_middle(reading.normalized)
        fresh = 0 <= age <= self.max_age_s
        previous = self._last_timestamp

        reason: str | None = None
        if not fresh:
            reason = (
                "observation timestamp is in the future"
                if age < 0
                else f"observation is stale ({age:.3f}s old)"
            )
        elif previous is not None and timestamp <= previous:
            reason = "duplicate or non-increasing observation timestamp"
        elif previous is not None and (timestamp - previous) > self.max_age_s:
            reason = (
                f"sampling gap of {timestamp - previous:.3f}s between observations"
            )

        if reason is not None:
            # Stale, replayed, or gapped observations cannot confirm anything.
            self._run_edge = None
            self._run_count = 0
        elif edge is self._run_edge:
            self._run_count += 1
        else:
            self._run_edge = edge
            self._run_count = 1
        # A rejected replay must never lower the high-water mark. A fresh
        # frame after a sampling gap establishes the new baseline, allowing
        # subsequent consecutive frames to reacquire normally. Future/stale
        # evidence never poisons that baseline.
        if fresh and (previous is None or timestamp > previous):
            self._last_timestamp = timestamp

        stable = reason is None and self._run_count >= self.stable_samples
        stable_edge = edge if stable else EdgeState.UNKNOWN
        black_count = sum(1 for value in reading.normalized if value)
        self.last_rejection = reason
        detection = EdgeDetection(
            raw=reading.raw,
            normalized=reading.normalized,
            edge=edge,
            timestamp=timestamp,
            stable=stable,
            stable_edge=stable_edge,
            stable_count=self._run_count,
            age_s=age,
            outer_left=bool(reading.normalized[0]),
            outer_right=bool(reading.normalized[3]),
            junction=black_count >= self.junction_channels,
            rejected=reason is not None,
            reject_reason=reason,
        )
        self._last = detection
        return detection

    def _validate_reading(self, reading: LineReading) -> None:
        if not isinstance(reading, LineReading):
            raise SensorError(f"expected a LineReading, got {type(reading).__name__}")
        timestamp = reading.timestamp
        if (
            isinstance(timestamp, bool)
            or not isinstance(timestamp, (int, float))
            or not math.isfinite(timestamp)
        ):
            raise SensorError(
                f"observation timestamp must be finite, got {timestamp!r}"
            )
        for name, channels in (
            ("raw", reading.raw),
            ("normalized", reading.normalized),
        ):
            if len(channels) != 4 or any(
                isinstance(value, bool) or value not in (0, 1)
                for value in channels
            ):
                raise SensorError(
                    f"{name} channels must be four 0/1 values, got {list(channels)!r}"
                )


def detect_edge(
    sensor_values: LineReading | int | Sequence[int],
    *,
    detector: EdgeDetector | None = None,
    config: SensorConfig | None = None,
    timestamp: float | None = None,
) -> EdgeDetection:
    """Classify one observation, optionally through a debouncing detector.

    ``sensor_values`` accepts a :class:`~waferbot.hardware.sensors.LineReading`,
    a raw sensor byte, or four raw channel bits ordered S1..S4.
    """
    reading = _coerce_reading(sensor_values, config, timestamp)
    active = detector or EdgeDetector()
    return active.update(reading)


def edge_byte_for(
    edge: EdgeState, config: SensorConfig | None = None, *, outer: bool = False
) -> int:
    """Raw sensor byte that reports ``edge`` under the given configuration.

    Used by diagnostics, mock sessions, and tests. ``outer=True`` also puts
    black under the outer channel on the same side, which is how junctions and
    wide tape look.
    """
    config = config or SensorConfig()
    # Respect the configured polarity: a raw 0 may mean black or white.
    black, white = (0, 1) if config.black_is_raw_zero else (1, 0)
    raw = [white, white, white, white]
    if edge is EdgeState.BLACK_LEFT:
        raw[1] = black
        if outer:
            raw[0] = black
    elif edge is EdgeState.BLACK_RIGHT:
        raw[2] = black
        if outer:
            raw[3] = black
    elif edge is EdgeState.BOTH_BLACK:
        raw[1] = black
        raw[2] = black
    elif edge is EdgeState.BOTH_WHITE:
        pass
    else:
        raise ValueError(f"{edge.value} has no sensor byte")
    return sum(
        bit << bit_index for bit, bit_index in zip(raw, config.bit_for_channel)
    )


# -- oriented boundary estimation -------------------------------------------

#: Array coordinates of S1..S4 in units of one sensor pitch.
SENSOR_POSITIONS: tuple[float, float, float, float] = (0.0, 1.0, 2.0, 3.0)
#: Midpoint between S2 and S3: the correctly centred boundary position.
CENTRE_POSITION = 1.5
#: Magnitude used when the oriented boundary is outside the sensor array.
SATURATED_ERROR_PITCHES = 2.0


@dataclass(frozen=True)
class BoundaryEstimate:
    """Where the followed tape boundary sits relative to the sensor array.

    ``error_pitches`` is signed in units of one sensor pitch, positive meaning
    "the boundary is to the robot's right, steer right".
    """

    edge: EdgeState
    error_pitches: float | None
    confidence: float
    position: float | None
    reason: str
    transitions: tuple[tuple[float, str], ...] = ()

    @property
    def usable(self) -> bool:
        return self.error_pitches is not None and self.confidence > 0.0

    def as_dict(self) -> dict[str, object]:
        return {
            "edge": self.edge.value,
            "error_pitches": self.error_pitches,
            "confidence": self.confidence,
            "position": self.position,
            "reason": self.reason,
            "transitions": [[pos, kind] for pos, kind in self.transitions],
        }


def _channel_transitions(normalized: Sequence[int]) -> tuple[tuple[float, str], ...]:
    """Adjacent black/white transitions as ``(position, kind)`` pairs.

    ``kind`` is ``"bw"`` for a black-to-white step (reading left to right) and
    ``"wb"`` for white-to-black.
    """
    transitions = []
    for index in range(3):
        left = int(normalized[index])
        right = int(normalized[index + 1])
        if left == 1 and right == 0:
            transitions.append((index + 0.5, "bw"))
        elif left == 0 and right == 1:
            transitions.append((index + 0.5, "wb"))
    return tuple(transitions)


def _black_runs(normalized: Sequence[int]) -> tuple[tuple[int, int], ...]:
    """Contiguous runs of black channels as inclusive ``(start, end)`` pairs."""
    runs: list[tuple[int, int]] = []
    start: int | None = None
    for index, value in enumerate(normalized):
        if value and start is None:
            start = index
        elif not value and start is not None:
            runs.append((start, index - 1))
            start = None
    if start is not None:
        runs.append((start, len(normalized) - 1))
    return tuple(runs)


def estimate_boundary(
    normalized: Sequence[int],
    target_edge: EdgeState | str,
    *,
    max_error_pitches: float = SATURATED_ERROR_PITCHES,
) -> BoundaryEstimate:
    """Estimate the followed boundary from four normalised channels.

    ``BLACK_LEFT`` follows the black-to-white transition (black on the left of
    the boundary); ``BLACK_RIGHT`` follows the white-to-black transition. A
    correctly centred pattern gives exactly zero error: ``1100`` and ``0100``
    for ``BLACK_LEFT``, ``0011`` and ``0010`` for ``BLACK_RIGHT``.

    The estimator never switches to the opposite boundary: a pattern that only
    shows the *other* transition saturates with the sign that steers back toward
    the followed edge, and reports reduced confidence.
    """
    edge = target_edge if isinstance(target_edge, EdgeState) else EdgeState.from_name(target_edge)
    if edge not in (EdgeState.BLACK_LEFT, EdgeState.BLACK_RIGHT):
        raise ValueError(f"{edge.value} is not a followable boundary")
    if len(normalized) != 4:
        raise SensorError(f"expected four normalised channels, got {list(normalized)!r}")
    channels = []
    for value in normalized:
        if isinstance(value, bool) or not isinstance(value, int) or value not in (0, 1):
            raise SensorError(f"normalised channels must be 0/1, got {value!r}")
        channels.append(value)

    want = "bw" if edge is EdgeState.BLACK_LEFT else "wb"
    transitions = _channel_transitions(channels)
    runs = _black_runs(channels)

    def saturated(error_sign: float, reason: str, confidence: float) -> BoundaryEstimate:
        return BoundaryEstimate(
            edge=edge,
            error_pitches=error_sign * float(max_error_pitches),
            confidence=confidence,
            position=None,
            reason=reason,
            transitions=transitions,
        )

    if not runs:
        # No tape under the array. The followed tape is assumed to still be on
        # the side the operator mounted it: steer back toward it, with low
        # confidence so the follower's ambiguity bound stays in control.
        return saturated(
            -1.0 if edge is EdgeState.BLACK_LEFT else 1.0,
            "uniform all-white reading (no tape under the array)",
            0.35,
        )

    if len(runs) == 1:
        start, end = runs[0]
        if edge is EdgeState.BLACK_LEFT:
            # Follow the right edge of the black band: black on its left.
            if end == len(channels) - 1:
                return saturated(
                    1.0,
                    "black band continues past S4; the followed boundary is to "
                    "the right of the array",
                    0.35,
                )
            position = end + 0.5
        else:
            # Follow the left edge of the black band: black on its right.
            if start == 0:
                return saturated(
                    -1.0,
                    "black band continues past S1; the followed boundary is to "
                    "the left of the array",
                    0.35,
                )
            position = start - 0.5
        confidence = 1.0
        reason = "single oriented boundary"
    else:
        # More than one black run: contradictory evidence. Prefer the wanted
        # transition closest to the centre, at low confidence.
        wanted = [pos for pos, kind in transitions if kind == want]
        if not wanted:
            return saturated(
                1.0 if edge is EdgeState.BLACK_LEFT else -1.0,
                "multiple black runs without an oriented transition",
                0.2,
            )
        position = min(wanted, key=lambda pos: abs(pos - CENTRE_POSITION))
        confidence = 0.25
        reason = "multiple black runs (contradictory evidence)"

    error = position - CENTRE_POSITION
    error = max(-float(max_error_pitches), min(float(max_error_pitches), error))
    return BoundaryEstimate(
        edge=edge,
        error_pitches=error,
        confidence=confidence,
        position=position,
        reason=reason,
        transitions=transitions,
    )


__all__ = [
    "BoundaryEstimate",
    "EdgeDetection",
    "EdgeDetector",
    "EdgeState",
    "CENTRE_POSITION",
    "SENSOR_POSITIONS",
    "classify_middle",
    "detect_edge",
    "edge_byte_for",
    "estimate_boundary",
]
