"""Process-safe motion ownership and stop coordination.

Two independent guards keep a second command from being ignored by a running
controller:

* **Ownership**: a motion session takes an exclusive ``flock`` on
  ``owner.lock`` and records itself in ``owner.json``. A second motion session
  refuses to start while one is live, so two controllers cannot fight over the
  bus.
* **Stop latch**: ``waferbot stop`` writes ``stop.json`` atomically. Every
  motion command checks that file immediately before driving, so a running
  controller stops and refuses to move again; the stop latch survives until it is
  explicitly cleared.

``bus.lock`` serialises I2C traffic between processes: the owner holds it around
each motor command and the stop command takes it before writing stop blocks, so
a stop cannot land in the middle of a four-wheel write.

POSIX only (Linux/Raspberry Pi OS and macOS), which matches the target hardware.
"""

from __future__ import annotations

import contextlib
import fcntl
import json
import os
import threading
import time
from collections.abc import Iterator
from dataclasses import dataclass
from pathlib import Path

from .errors import SafetyError

DEFAULT_RUNTIME_DIRNAME = "waferbot"


def default_runtime_dir() -> Path:
    override = os.environ.get("WAFERBOT_RUNTIME_DIR")
    if override:
        return Path(override)
    return Path.home() / ".cache" / DEFAULT_RUNTIME_DIRNAME


@dataclass(frozen=True)
class SessionInfo:
    pid: int
    command: str
    started_at: float
    runtime_dir: str

    def as_dict(self) -> dict[str, object]:
        return {
            "pid": self.pid,
            "command": self.command,
            "started_at": self.started_at,
            "runtime_dir": self.runtime_dir,
        }


@dataclass(frozen=True)
class StopRequest:
    pid: int
    reason: str
    requested_at: float

    def as_dict(self) -> dict[str, object]:
        return {
            "pid": self.pid,
            "reason": self.reason,
            "requested_at": self.requested_at,
        }


class ProcessGuard:
    """Ownership, stop latch, and cross-process bus serialisation."""

    def __init__(
        self,
        runtime_dir: str | Path | None = None,
        *,
        session_name: str = "default",
        clock=time.time,
    ) -> None:
        base = Path(runtime_dir) if runtime_dir else default_runtime_dir()
        if session_name and session_name != "default":
            base = base / session_name
        self.runtime_dir = base
        self.clock = clock
        self.owner_lock_path = self.runtime_dir / "owner.lock"
        self.owner_path = self.runtime_dir / "owner.json"
        self.stop_path = self.runtime_dir / "stop.json"
        self.bus_lock_path = self.runtime_dir / "bus.lock"
        self._owner_fd: int | None = None
        # Per-thread nesting depth so a stop taken inside an existing
        # transaction cannot deadlock on a second flock of the same file.
        self._local = threading.local()

    # -- setup --------------------------------------------------------------

    def ensure_dir(self) -> None:
        self.runtime_dir.mkdir(parents=True, exist_ok=True)

    @property
    def holds_ownership(self) -> bool:
        return self._owner_fd is not None

    # -- ownership ----------------------------------------------------------

    def acquire_ownership(self, command: str) -> SessionInfo:
        """Take the motion-session lock, or raise if another session is live."""
        self.ensure_dir()
        if self._owner_fd is not None:
            raise SafetyError("this process already owns a motion session")
        fd = os.open(self.owner_lock_path, os.O_RDWR | os.O_CREAT, 0o644)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            os.close(fd)
            existing = self.session()
            detail = (
                f"pid {existing.pid} running {existing.command!r}"
                if existing
                else "another process"
            )
            raise SafetyError(
                f"another motion session is already running ({detail}); "
                "stop it before starting a new one"
            ) from exc
        self._owner_fd = fd
        info = SessionInfo(
            pid=os.getpid(),
            command=command,
            started_at=self.clock(),
            runtime_dir=str(self.runtime_dir),
        )
        _atomic_write_json(self.owner_path, info.as_dict())
        return info

    def release_ownership(self) -> None:
        if self._owner_fd is None:
            return
        try:
            fcntl.flock(self._owner_fd, fcntl.LOCK_UN)
        finally:
            os.close(self._owner_fd)
            self._owner_fd = None
        if self.session() is not None:
            with contextlib.suppress(FileNotFoundError):
                self.owner_path.unlink()

    def session(self) -> SessionInfo | None:
        data = _read_json(self.owner_path)
        if not data:
            return None
        try:
            return SessionInfo(
                pid=int(data["pid"]),
                command=str(data.get("command", "")),
                started_at=float(data.get("started_at", 0.0)),
                runtime_dir=str(data.get("runtime_dir", self.runtime_dir)),
            )
        except (KeyError, TypeError, ValueError):
            return None

    def session_is_live(self) -> bool:
        """True when the recorded owner process still exists and is ours."""
        info = self.session()
        if info is None:
            return False
        try:
            os.kill(info.pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def owner_lock_available(self) -> bool:
        """True when no other process currently holds the motion-session lock."""
        self.ensure_dir()
        fd = os.open(self.owner_lock_path, os.O_RDWR | os.O_CREAT, 0o644)
        try:
            try:
                fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                return False
            with contextlib.suppress(OSError):
                fcntl.flock(fd, fcntl.LOCK_UN)
            return True
        finally:
            os.close(fd)

    # -- stop latch ---------------------------------------------------------

    def stop_requested(self) -> bool:
        return self.stop_path.exists()

    def stop_request(self) -> StopRequest | None:
        data = _read_json(self.stop_path)
        if not data:
            return None
        try:
            return StopRequest(
                pid=int(data.get("pid", 0)),
                reason=str(data.get("reason", "")),
                requested_at=float(data.get("requested_at", 0.0)),
            )
        except (TypeError, ValueError):
            return None

    def request_stop(self, reason: str) -> StopRequest:
        """Atomically latch a stop request. Safe to call from any process."""
        self.ensure_dir()
        request = StopRequest(
            pid=os.getpid(), reason=reason, requested_at=self.clock()
        )
        _atomic_write_json(self.stop_path, request.as_dict())
        return request

    def clear_stop_request(self) -> bool:
        """Remove the stop latch. Returns True when a latch existed."""
        had = self.stop_path.exists()
        with contextlib.suppress(FileNotFoundError):
            self.stop_path.unlink()
        return had

    # -- bus serialisation --------------------------------------------------

    @contextlib.contextmanager
    def bus_guard(self, timeout_s: float = 1.0) -> Iterator[None]:
        """Exclusive cross-process lock around a bus transaction sequence.

        Re-entrant within one thread (nested stop paths reuse the held lock);
        a second process, or another thread in this process, still blocks until
        the guard is released.
        """
        depth = getattr(self._local, "depth", 0)
        if depth > 0:
            self._local.depth = depth + 1
            try:
                yield
            finally:
                self._local.depth -= 1
            return
        self.ensure_dir()
        fd = os.open(self.bus_lock_path, os.O_RDWR | os.O_CREAT, 0o644)
        deadline = time.monotonic() + max(0.0, timeout_s)
        try:
            while True:
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    break
                except OSError as exc:
                    if time.monotonic() >= deadline:
                        raise TimeoutError(
                            "timed out waiting for the I2C bus lock"
                        ) from exc
                    time.sleep(0.01)
            self._local.depth = 1
            yield
        finally:
            self._local.depth = 0
            with contextlib.suppress(OSError):
                fcntl.flock(fd, fcntl.LOCK_UN)
            os.close(fd)

    @property
    def holds_bus_lock(self) -> bool:
        return getattr(self._local, "depth", 0) > 0


def _atomic_write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + f".tmp{os.getpid()}")
    with tmp.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)


def _read_json(path: Path) -> dict | None:
    try:
        with path.open(encoding="utf-8") as handle:
            data = json.load(handle)
    except FileNotFoundError:
        return None
    except (json.JSONDecodeError, OSError):
        return None
    return data if isinstance(data, dict) else None


__all__ = [
    "ProcessGuard",
    "SessionInfo",
    "StopRequest",
    "default_runtime_dir",
]
