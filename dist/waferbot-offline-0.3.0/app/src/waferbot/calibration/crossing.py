"""Crossing-angle sweep: measure which angles actually complete a switch.

The geometry module produces a *bound*; this tool produces evidence. Each
candidate angle is attempted a configurable number of times and the success
rate, switch time, and final edge state are written to CSV so an operator can
pick a conservative value.
"""

from __future__ import annotations

import csv
import time
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass
class CrossingSweepEntry:
    angle_deg: float
    attempts: int = 0
    successes: int = 0
    total_time_s: float = 0.0
    final_edges: list[str] = field(default_factory=list)
    reasons: list[str] = field(default_factory=list)

    @property
    def success_rate(self) -> float:
        return self.successes / self.attempts if self.attempts else 0.0

    @property
    def mean_time_s(self) -> float:
        return self.total_time_s / self.attempts if self.attempts else 0.0

    @property
    def final_edge(self) -> str | None:
        return self.final_edges[-1] if self.final_edges else None

    def as_dict(self) -> dict[str, Any]:
        return {
            "angle_deg": self.angle_deg,
            "attempts": self.attempts,
            "successes": self.successes,
            "success_rate": self.success_rate,
            "mean_time_s": self.mean_time_s,
            "final_edge": self.final_edge,
            "reasons": self.reasons,
        }


@dataclass
class CrossingSweepReport:
    entries: list[CrossingSweepEntry] = field(default_factory=list)
    csv_path: str | None = None
    confirmed: bool = False
    notes: list[str] = field(default_factory=list)

    @property
    def best_angle_deg(self) -> float | None:
        # Only angles that actually succeeded can be recommended.
        usable = [
            entry for entry in self.entries if entry.attempts and entry.successes
        ]
        if not usable:
            return None
        return max(usable, key=lambda entry: (entry.success_rate, -entry.angle_deg)).angle_deg

    def as_dict(self) -> dict[str, Any]:
        return {
            "csv_path": self.csv_path,
            "best_angle_deg": self.best_angle_deg,
            "confirmed": self.confirmed,
            "entries": [entry.as_dict() for entry in self.entries],
            "notes": self.notes,
        }


class CrossingSweep:
    """Runs a switch attempt for each candidate angle and records the outcome."""

    def __init__(
        self,
        attempt: Callable[[float], Any],
        *,
        csv_path: str | Path | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.attempt = attempt
        self.csv_path = Path(csv_path) if csv_path else None
        self.clock = clock

    def run(
        self,
        angles: Sequence[float],
        *,
        attempts_per_angle: int = 3,
        minimum_success_rate: float = 1.0,
    ) -> CrossingSweepReport:
        if attempts_per_angle < 1:
            raise ValueError("attempts_per_angle must be >= 1")
        report = CrossingSweepReport(csv_path=str(self.csv_path) if self.csv_path else None)
        report.notes.append(
            "Each row is one real attempt. Confirm the destination location after "
            "every crossing; do not trust the opposite edge alone."
        )
        rows: list[dict[str, Any]] = []
        for angle in angles:
            entry = CrossingSweepEntry(angle_deg=float(angle))
            for attempt_index in range(attempts_per_angle):
                started = self.clock()
                result = self.attempt(float(angle))
                elapsed = self.clock() - started
                entry.attempts += 1
                entry.total_time_s += elapsed
                completed = bool(getattr(result, "completed", False))
                if completed:
                    entry.successes += 1
                entry.reasons.append(str(getattr(result, "reason", "")))
                final_edge = getattr(result, "target_edge", None)
                entry.final_edges.append(
                    getattr(final_edge, "value", str(final_edge))
                    if completed
                    else "NOT_REACHED"
                )
                rows.append(
                    {
                        "kind": "attempt",
                        "angle_deg": entry.angle_deg,
                        "attempt": attempt_index + 1,
                        "completed": completed,
                        "elapsed_s": f"{elapsed:.3f}",
                        "final_edge": entry.final_edges[-1],
                        "reason": entry.reasons[-1],
                    }
                )
            report.entries.append(entry)
        for entry in report.entries:
            rows.append(
                {
                    "kind": "summary",
                    "angle_deg": entry.angle_deg,
                    "attempt": entry.attempts,
                    "completed": entry.successes,
                    "elapsed_s": f"{entry.mean_time_s:.3f}",
                    "final_edge": entry.final_edge or "NOT_REACHED",
                    "reason": f"success_rate={entry.success_rate:.3f}",
                }
            )
        report.confirmed = bool(report.entries) and all(
            entry.success_rate >= minimum_success_rate for entry in report.entries
        )
        if self.csv_path is not None:
            self._write_csv(rows)
        return report

    def _write_csv(self, rows: Iterable[dict[str, Any]]) -> None:
        rows = list(rows)
        assert self.csv_path is not None
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = [
            "kind",
            "angle_deg",
            "attempt",
            "completed",
            "elapsed_s",
            "final_edge",
            "reason",
        ]
        with self.csv_path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(row)


__all__ = ["CrossingSweep", "CrossingSweepEntry", "CrossingSweepReport"]
