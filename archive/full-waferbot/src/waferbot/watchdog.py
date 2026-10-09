"""Independent motion watchdog.

`Robot.tick()` only helps if the control loop that owns the robot is still
calling it. A stalled, blocked, or crashed controller would never call `tick()`
again, so the wheels would keep their last command forever. This module runs a
separate daemon thread that watches a monotonic deadline and fires a stop when
the deadline passes without a refresh.

The callback runs on the watchdog thread. It is expected to take the shared bus
lock, stop the wheels, and latch a fault; exceptions are captured in
``last_error`` because a thread cannot propagate them to the caller.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable


class MotionWatchdog:
    """Deadline checker running on its own thread."""

    def __init__(
        self,
        on_timeout: Callable[[], None],
        *,
        clock: Callable[[], float] = time.monotonic,
        poll_interval_s: float = 0.05,
        name: str = "waferbot-watchdog",
    ) -> None:
        if poll_interval_s <= 0:
            raise ValueError("poll_interval_s must be positive")
        self._on_timeout = on_timeout
        self._clock = clock
        self._poll_interval_s = float(poll_interval_s)
        self._name = name
        self._lock = threading.Lock()
        self._deadline: float | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self.timeout_count = 0
        self.last_error: BaseException | None = None

    # -- inspection ---------------------------------------------------------

    @property
    def poll_interval_s(self) -> float:
        return self._poll_interval_s

    @property
    def deadline(self) -> float | None:
        with self._lock:
            return self._deadline

    @property
    def running(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    def expired(self) -> bool:
        deadline = self.deadline
        return deadline is not None and self._clock() >= deadline

    def consume_expired(self) -> bool:
        """Atomically claim an expired deadline.

        Returns ``False`` while the deadline is live (or unset), so a refresh
        that lands before this call cannot be mistaken for an expiry, and a
        refresh that lands after it re-arms a fresh deadline for the next check.
        """
        with self._lock:
            if self._deadline is None or self._clock() < self._deadline:
                return False
            self._deadline = None
            return True

    # -- control ------------------------------------------------------------

    def start(self) -> None:
        """Start the checker thread if it is not already running."""
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._run, name=self._name, daemon=True
            )
            thread = self._thread
        thread.start()

    def stop(self, *, join_timeout_s: float = 1.0) -> None:
        """Stop the checker thread. Does not touch the motors."""
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(join_timeout_s)
        self._thread = None

    def refresh(self, timeout_s: float) -> None:
        """Arm the deadline ``timeout_s`` seconds from now."""
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        with self._lock:
            self._deadline = self._clock() + float(timeout_s)

    def cancel(self) -> None:
        """Disarm the deadline without stopping the thread."""
        with self._lock:
            self._deadline = None

    # -- internals ----------------------------------------------------------

    def _run(self) -> None:
        while not self._stop_event.wait(self._poll_interval_s):
            if not self.consume_expired():
                continue
            self.timeout_count += 1
            try:
                self._on_timeout()
            except BaseException as exc:  # pragma: no cover - defensive
                self.last_error = exc


__all__ = ["MotionWatchdog"]
