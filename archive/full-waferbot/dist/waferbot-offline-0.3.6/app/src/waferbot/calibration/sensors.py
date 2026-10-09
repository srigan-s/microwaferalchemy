"""Sensor diagnostics and channel-order calibration.

The calibration is differential: it measures an all-white baseline first, then
asks the operator to cover exactly one physical channel at a time. A channel is
only accepted when every sample differs from the baseline in the *same single
bit*, which rejects mixed, noisy, or multi-channel frames. Polarity is inferred
from the direction of the change, so either firmware polarity can be detected.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from ..hardware.sensors import LineReading

#: Physical channel labels, left to right as seen from behind the robot.
CHANNEL_LABELS = (
    "S1 (leftmost)",
    "S2 (left middle)",
    "S3 (right middle)",
    "S4 (rightmost)",
)

#: A channel's covered sample changed 1 -> 0, i.e. raw 0 means black.
POLARITY_BLACK_LOW = "black_is_raw_zero"
#: A channel's covered sample changed 0 -> 1, i.e. raw 1 means black.
POLARITY_BLACK_HIGH = "black_is_raw_one"


@dataclass(frozen=True)
class SensorSample:
    raw_byte: int
    raw: tuple[int, int, int, int]
    normalized: tuple[int, int, int, int]
    timestamp: float

    @classmethod
    def from_reading(cls, reading: LineReading) -> "SensorSample":
        return cls(
            raw_byte=reading.raw_byte,
            raw=reading.raw,
            normalized=reading.normalized,
            timestamp=reading.timestamp,
        )

    def as_dict(self) -> dict[str, Any]:
        return {
            "raw_byte": self.raw_byte,
            "raw": list(self.raw),
            "normalized": list(self.normalized),
            "timestamp": self.timestamp,
        }


@dataclass
class SensorCalibrationReport:
    samples: list[SensorSample] = field(default_factory=list)
    baseline_samples: list[SensorSample] = field(default_factory=list)
    channel_bits: dict[str, int | None] = field(default_factory=dict)
    channel_direction: dict[str, str | None] = field(default_factory=dict)
    suggested_bit_for_channel: list[int] | None = None
    suggested_black_is_raw_zero: bool | None = None
    confirmed: bool = False
    notes: list[str] = field(default_factory=list)

    @property
    def black_bits(self) -> dict[str, int | None]:
        """Backwards-compatible alias for the per-channel bit map."""
        return self.channel_bits

    def as_dict(self) -> dict[str, Any]:
        return {
            "confirmed": self.confirmed,
            "black_bits": self.channel_bits,
            "channel_direction": self.channel_direction,
            "suggested_bit_for_channel": self.suggested_bit_for_channel,
            "suggested_black_is_raw_zero": self.suggested_black_is_raw_zero,
            "baseline_samples": [s.as_dict() for s in self.baseline_samples],
            "sample_count": len(self.samples),
            "samples": [sample.as_dict() for sample in self.samples],
            "notes": self.notes,
        }


def read_samples(
    robot,
    count: int = 5,
    *,
    interval_s: float = 0.1,
    sleep: Callable[[float], None] = time.sleep,
) -> list[SensorSample]:
    """Read raw sensor frames without touching the motors."""
    if count < 1:
        raise ValueError("count must be >= 1")
    samples: list[SensorSample] = []
    for index in range(count):
        samples.append(SensorSample.from_reading(robot.read_line_sensors()))
        if index + 1 < count and interval_s > 0:
            sleep(interval_s)
    return samples


def format_samples(samples: list[SensorSample]) -> str:
    """Human-readable diagnostic table for ``waferbot sensors``."""
    lines = ["raw_byte  raw(S1..S4)   BLACK(1)/WHITE(0)   timestamp"]
    for sample in samples:
        raw = " ".join(str(value) for value in sample.raw)
        norm = " ".join(str(value) for value in sample.normalized)
        lines.append(
            f"0x{sample.raw_byte:02X}      {raw}           {norm}        "
            f"{sample.timestamp:.3f}"
        )
    return "\n".join(lines)


def _common_bits(samples: list[SensorSample]) -> int | None:
    """A single baseline byte when every sample agrees, else ``None``."""
    if not samples:
        return None
    first = samples[0].raw_byte
    if all(sample.raw_byte == first for sample in samples):
        return first
    return None


def _single_bit_change(baseline: int, value: int) -> tuple[int, int] | None:
    """Return ``(bit, new_value)`` when exactly one bit differs."""
    difference = baseline ^ value
    if difference == 0 or (difference & (difference - 1)) != 0:
        return None
    bit = difference.bit_length() - 1
    return bit, (value >> bit) & 1


def calibrate_channels(
    robot,
    *,
    ask: Callable[[str], str],
    samples_per_channel: int = 5,
    interval_s: float = 0.1,
    sleep: Callable[[float], None] = time.sleep,
) -> SensorCalibrationReport:
    """Determine bit order and polarity from a baseline plus one-channel covers."""
    report = SensorCalibrationReport()
    report.notes.append(
        "Motors stay stopped for the whole sensor calibration; move the tape by hand."
    )
    robot.stop()

    answer = str(
        ask(
            "Place the whole sensor array over plain WHITE floor (no tape under "
            "any channel), then press enter (or type skip to abort)."
        )
    ).strip().lower()
    if answer == "skip":
        report.notes.append("baseline step skipped; calibration cannot be confirmed")
        return report

    baseline_samples = read_samples(
        robot, samples_per_channel, interval_s=interval_s, sleep=sleep
    )
    report.baseline_samples = baseline_samples
    baseline = _common_bits(baseline_samples)
    if baseline is None:
        report.notes.append(
            "baseline was not stable across samples (check lighting and "
            "reflections); re-run the calibration"
        )
        return report
    report.notes.append(f"white baseline byte 0x{baseline:02X}")

    directions: list[str] = []
    for label in CHANNEL_LABELS:
        answer = str(
            ask(
                f"Place black tape under ONLY {label} and keep the other three on "
                "white floor. Press enter when ready (or type skip)."
            )
        ).strip().lower()
        if answer == "skip":
            report.channel_bits[label] = None
            report.channel_direction[label] = None
            continue
        samples = read_samples(
            robot, samples_per_channel, interval_s=interval_s, sleep=sleep
        )
        report.samples.extend(samples)

        changes = []
        for sample in samples:
            change = _single_bit_change(baseline, sample.raw_byte)
            if change is None:
                changes = []
                break
            changes.append(change)
        if not changes or len({bit for bit, _value in changes}) != 1:
            report.channel_bits[label] = None
            report.channel_direction[label] = None
            report.notes.append(
                f"{label}: no consistent single-bit change from the baseline; "
                "re-run with only that channel covered"
            )
            continue

        bit = changes[0][0]
        new_values = {value for _bit, value in changes}
        if len(new_values) != 1:
            report.channel_bits[label] = None
            report.channel_direction[label] = None
            report.notes.append(
                f"{label}: bit {bit} flickered between 0 and 1; not confirmed"
            )
            continue

        new_value = new_values.pop()
        direction = POLARITY_BLACK_LOW if new_value == 0 else POLARITY_BLACK_HIGH
        report.channel_bits[label] = bit
        report.channel_direction[label] = direction
        directions.append(direction)

    bits = [report.channel_bits.get(label) for label in CHANNEL_LABELS]
    complete = all(bit is not None for bit in bits) and len(set(bits)) == 4
    consistent = len(set(directions)) == 1 and len(directions) == 4
    if complete and consistent:
        report.suggested_bit_for_channel = [int(bit) for bit in bits]  # type: ignore[arg-type]
        report.suggested_black_is_raw_zero = directions[0] == POLARITY_BLACK_LOW
        report.confirmed = True
    else:
        report.notes.append(
            "Could not confirm a clean one-bit-per-channel mapping with a single "
            "polarity; nothing is written to the configuration automatically."
        )
    return report


__all__ = [
    "CHANNEL_LABELS",
    "POLARITY_BLACK_HIGH",
    "POLARITY_BLACK_LOW",
    "SensorCalibrationReport",
    "SensorSample",
    "calibrate_channels",
    "format_samples",
    "read_samples",
]
