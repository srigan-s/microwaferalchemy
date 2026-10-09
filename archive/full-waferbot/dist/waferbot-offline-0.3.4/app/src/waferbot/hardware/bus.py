"""A re-entrant lock that makes a multi-transfer I2C command atomic.

A four-wheel command is four separate block writes. Without a shared lock the
independent watchdog (or a `waferbot stop` process) could interleave a stop
between those writes, leaving some wheels driving after the stop appeared to
have happened. Every transport access made by the driver and the sensor reader
is wrapped in this lock, so an observer sees either the whole command or none of
it.
"""

from __future__ import annotations

import contextlib
import threading
from collections.abc import Iterator


class BusLock:
    """Re-entrant lock shared by every component that talks to one I2C bus."""

    def __init__(self) -> None:
        self._lock = threading.RLock()

    def __enter__(self) -> "BusLock":
        self._lock.acquire()
        return self

    def __exit__(self, *_exc_info) -> None:
        self._lock.release()

    def acquire(self, blocking: bool = True, timeout: float = -1) -> bool:
        return self._lock.acquire(blocking, timeout)

    def release(self) -> None:
        self._lock.release()

    def locked(self) -> bool:
        return self._lock._is_owned()  # type: ignore[attr-defined]


@contextlib.contextmanager
def null_bus_guard(*_args, **_kwargs) -> Iterator[None]:
    """No-op stand-in used when no cross-process lock is configured."""
    yield


__all__ = ["BusLock", "null_bus_guard"]
