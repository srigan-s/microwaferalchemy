"""Single latest-sample IR poller for physical closed-loop following.

Only one reader thread exists per follower. It retains one sample, never an
unbounded queue, and sleeps to monotonic deadlines rather than busy-waiting.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from ..hardware.sensors import LineReading


@dataclass(frozen=True)
class PollSnapshot:
    sequence: int
    reading: LineReading | None
    observed_hz: float | None
    latency_ms: float | None
    missed_deadlines: int
    error: BaseException | None


class LatestIRPoller:
    def __init__(self, robot, *, rate_hz: float = 300.0) -> None:
        if not 1 <= rate_hz <= 1000:
            raise ValueError("IR polling rate must be 1..1000 Hz")
        self.robot = robot
        self.rate_hz = rate_hz
        self._period_ns = round(1e9 / rate_hz)
        self._condition = threading.Condition()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._sequence = 0
        self._reading: LineReading | None = None
        self._first_ns: int | None = None
        self._last_ns: int | None = None
        self._latency_ms: float | None = None
        self._missed = 0
        self._error: BaseException | None = None

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("IR poller already started")
        self._thread = threading.Thread(target=self._run, name="waferbot-ir-poll", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._condition:
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def latest_after(self, sequence: int, timeout_s: float) -> PollSnapshot:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._sequence <= sequence and self._error is None and not self._stop.is_set():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._condition.wait(remaining)
            observed = None
            if self._sequence > 1 and self._first_ns is not None and self._last_ns > self._first_ns:
                observed = (self._sequence - 1) * 1e9 / (self._last_ns - self._first_ns)
            return PollSnapshot(
                self._sequence, self._reading, observed, self._latency_ms,
                self._missed, self._error,
            )

    def _run(self) -> None:
        next_ns = time.monotonic_ns()
        while not self._stop.is_set():
            remaining = (next_ns - time.monotonic_ns()) / 1e9
            if remaining > 0 and self._stop.wait(remaining):
                break
            started = time.monotonic_ns()
            try:
                reading = self.robot.read_line_sensors()
            except BaseException as exc:
                with self._condition:
                    self._error = exc
                    self._condition.notify_all()
                break
            finished = time.monotonic_ns()
            with self._condition:
                if self._first_ns is None:
                    self._first_ns = finished
                self._last_ns = finished
                self._sequence += 1
                self._reading = reading
                self._latency_ms = (finished - started) / 1e6
                self._condition.notify_all()
            next_ns += self._period_ns
            if finished >= next_ns:
                skipped = (finished - next_ns) // self._period_ns + 1
                self._missed += skipped
                next_ns += skipped * self._period_ns
