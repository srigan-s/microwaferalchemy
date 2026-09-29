"""CSV telemetry with the fields the project asked for.

Rows are event rows: sensor reads, motor commands, controller decisions,
localization updates, and faults. A single row carries whatever is known at
that moment; empty columns are honest "not measured here" values rather than
placeholders.

``estimated_speed_mps`` stays empty unless a measured ``counts_to_mps`` factor
is configured. There is no pretended conversion.
"""

from __future__ import annotations

import csv
import threading
import time
from collections.abc import Callable, Mapping
from pathlib import Path

CSV_FIELDS: tuple[str, ...] = (
    "timestamp",
    "event",
    "raw",
    "normalized",
    "detected_edge",
    "target_edge",
    "stable",
    "current_node",
    "target_node",
    "phase",
    "motor_m1",
    "motor_m2",
    "motor_m3",
    "motor_m4",
    "motor_context",
    "estimated_speed_pwm",
    "estimated_speed_mps",
    "fault",
    "note",
)


class TelemetryLogger:
    """Append-only CSV logger."""

    def __init__(
        self,
        path: str | Path,
        *,
        counts_to_mps: float | None = None,
        clock: Callable[[], float] = time.monotonic,
        flush_every: int = 1,
    ) -> None:
        if counts_to_mps is not None and counts_to_mps <= 0:
            raise ValueError("counts_to_mps must be positive when provided")
        self.path = Path(path)
        self.counts_to_mps = counts_to_mps
        self.rows_written = 0
        self._clock = clock
        self._flush_every = max(1, int(flush_every))
        self._lock = threading.Lock()
        self._closed = False
        self.path.parent.mkdir(parents=True, exist_ok=True)
        is_new = not self.path.exists() or self.path.stat().st_size == 0
        self._handle = self.path.open("a", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._handle, fieldnames=list(CSV_FIELDS))
        if is_new:
            self._writer.writeheader()

    # -- lifecycle ----------------------------------------------------------

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            handle = getattr(self, "_handle", None)
            if handle is not None and not handle.closed:
                handle.flush()
                handle.close()

    def __enter__(self) -> "TelemetryLogger":
        return self

    def __exit__(self, *_exc_info) -> None:
        self.close()

    # -- generic logging ----------------------------------------------------

    def log(self, event: str, **fields) -> None:
        with self._lock:
            if self._closed:
                # A closed logger must not raise from a fault path; the runbook
                # documents that rows after close are dropped.
                return
            row = {name: "" for name in CSV_FIELDS}
            row["timestamp"] = f"{self._clock():.6f}"
            row["event"] = event
            notes: list[str] = []
            for key, value in fields.items():
                if key not in CSV_FIELDS:
                    notes.append(f"{key}={value}")
                    continue
                if isinstance(value, (list, tuple)):
                    row[key] = " ".join(str(item) for item in value)
                elif isinstance(value, bool):
                    row[key] = "1" if value else "0"
                elif value is None:
                    row[key] = ""
                else:
                    row[key] = value
            if notes:
                existing = row["note"]
                row["note"] = "; ".join(filter(None, [existing] + notes))
            self._writer.writerow(row)
            self.rows_written += 1
            if self.rows_written % self._flush_every == 0:
                self._handle.flush()

    # -- convenience --------------------------------------------------------

    def sensor_reading(self, reading) -> None:
        self.log(
            "sensor",
            raw=reading.raw,
            normalized=reading.normalized,
        )

    def motor_command(self, values, *, context: str = "") -> None:
        values = list(values)
        row: dict[str, object] = {"motor_context": context}
        for index, value in enumerate(values[:4], start=1):
            row[f"motor_m{index}"] = value
        if values:
            mean = sum(abs(int(v)) for v in values) / len(values)
            row["estimated_speed_pwm"] = f"{mean:.2f}"
            if self.counts_to_mps is not None:
                row["estimated_speed_mps"] = f"{mean * self.counts_to_mps:.4f}"
        self.log("motor", **row)

    def control(
        self,
        *,
        event: str = "control",
        detected_edge=None,
        target_edge=None,
        current_node=None,
        target_node=None,
        phase=None,
        stable=None,
        raw=None,
        normalized=None,
        fault=None,
        note=None,
    ) -> None:
        self.log(
            event,
            detected_edge=detected_edge,
            target_edge=target_edge,
            current_node=current_node,
            target_node=target_node,
            phase=phase,
            stable=stable,
            raw=raw,
            normalized=normalized,
            fault=fault,
            note=note,
        )

    def localization(self, localization, *, expected: str | None = None) -> None:
        self.log(
            "localization",
            current_node=localization.node_id,
            target_node=expected,
            note=f"source={localization.source} confidence={localization.confidence}",
        )

    def fault(self, record) -> None:
        self.log(
            "fault",
            fault=record.code.value,
            note=record.message,
        )

    def stop(self, context: str = "stop") -> None:
        self.log("stop", motor_context=context)

    def switch_phase(self, phase: str, *, target_edge=None, note=None) -> None:
        self.log(
            "switch_phase",
            phase=phase,
            target_edge=target_edge,
            note=note,
        )


class NullTelemetry:
    """No-op telemetry used when logging is disabled."""

    rows_written = 0
    counts_to_mps = None

    def log(self, event: str, **fields) -> None:  # noqa: D401 - trivial
        return None

    def sensor_reading(self, reading) -> None:
        return None

    def motor_command(self, values, *, context: str = "") -> None:
        return None

    def control(self, **fields) -> None:
        return None

    def localization(self, localization, *, expected: str | None = None) -> None:
        return None

    def fault(self, record) -> None:
        return None

    def stop(self, context: str = "stop") -> None:
        return None

    def switch_phase(self, phase: str, *, target_edge=None, note=None) -> None:
        return None

    def close(self) -> None:
        return None

    def __enter__(self) -> "NullTelemetry":
        return self

    def __exit__(self, *_exc_info) -> None:
        return None


def open_telemetry(
    path: str | Path | None,
    *,
    counts_to_mps: float | None = None,
    clock: Callable[[], float] = time.monotonic,
):
    """Return a logger for ``path`` or a null logger when disabled."""
    if path is None:
        return NullTelemetry()
    return TelemetryLogger(path, counts_to_mps=counts_to_mps, clock=clock)


def read_rows(path: str | Path) -> list[dict[str, str]]:
    """Read back a telemetry CSV (used by tests and the runbook)."""
    with Path(path).open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


__all__ = [
    "CSV_FIELDS",
    "NullTelemetry",
    "TelemetryLogger",
    "open_telemetry",
    "read_rows",
]
