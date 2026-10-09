"""In-memory I2C transport for hardware-free tests and dry runs.

The mock records every transfer so tests can assert the exact protocol bytes,
and it can inject failures so fault handling is exercised without a Pi.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass, field

from ..config import RobotConfig
from ..errors import I2CError


def _default_write_error() -> Exception:
    return OSError("mock I2C write failure")


@dataclass
class MockI2CTransport:
    """A transport that answers reads from memory and records writes.

    Attributes:
        line_sensor_byte: byte returned for the line sensor register.
        line_sensor_length: frame length to return; set to a wrong value to
            exercise malformed-frame handling.
        fail_write_indices: 0-based indices into the write sequence that should
            raise. Index 2 means "the third write fails".
        write_error_factory: exception raised for a failing write.
        read_error: exception raised by every read when set.
    """

    line_sensor_byte: int = 0xFF
    line_sensor_length: int = 1
    #: Scripted sensor bytes, consumed one per read. The last entry repeats
    #: once the sequence is exhausted; an empty sequence falls back to
    #: ``line_sensor_byte``.
    line_sensor_sequence: list[int] = field(default_factory=list)
    fail_write_indices: set[int] = field(default_factory=set)
    #: When true every write fails, regardless of index.
    fail_all_writes: bool = False
    write_error_factory: Callable[[], Exception] = _default_write_error
    read_error: Exception | None = None
    #: When set, every stop block after the first ``fail_stops_after`` stops
    #: fails. Used to test that cleanup failures are reported honestly.
    fail_stops_after: int | None = None
    stop_writes_seen: int = 0
    #: Optional callable that supplies the next sensor byte. Takes precedence
    #: over ``line_sensor_sequence``/``line_sensor_byte`` when set.
    line_sensor_provider: Callable[[], int] | None = None
    #: Optional hooks used to test bus-lock ordering between threads.
    before_write: Callable[[int], None] | None = None
    before_read: Callable[[], None] | None = None
    writes: list[tuple[int, int, tuple[int, ...]]] = field(default_factory=list)
    reads: list[tuple[int, int, int]] = field(default_factory=list)
    write_attempts: int = 0
    read_attempts: int = 0
    closed: bool = False

    # -- I2CTransport protocol ---------------------------------------------

    def write_block(
        self, address: int, register: int, data: Sequence[int]
    ) -> None:
        # Index counts every attempt, including failed ones, so a test can say
        # "the third write fails" without the failure shifting later indices.
        index = self.write_attempts
        self.write_attempts += 1
        if self.before_write is not None:
            self.before_write(index)
        if (
            self.fail_stops_after is not None
            and len(data) == 3
            and int(data[2]) == 0
        ):
            if self.stop_writes_seen >= self.fail_stops_after:
                cause = OSError("mock stop failure")
                raise I2CError(f"mock stop write failed (index {index})") from cause
            self.stop_writes_seen += 1
        if self.fail_all_writes or index in self.fail_write_indices:
            cause = self.write_error_factory()
            raise I2CError(f"mock write failed (index {index}): {cause}") from cause
        self.writes.append((address, register, tuple(int(b) for b in data)))

    def read_block(self, address: int, register: int, length: int) -> list[int]:
        index = self.read_attempts
        self.read_attempts += 1
        if self.before_read is not None:
            self.before_read()
        self.reads.append((address, register, length))
        if self.read_error is not None:
            raise I2CError(f"mock read failed: {self.read_error}") from self.read_error
        value = self.current_line_sensor_byte(index)
        return [value] * max(0, self.line_sensor_length)

    def current_line_sensor_byte(self, index: int | None = None) -> int:
        """Byte the next (or ``index``-th) sensor read will return."""
        if self.line_sensor_provider is not None:
            return int(self.line_sensor_provider())
        if not self.line_sensor_sequence:
            return self.line_sensor_byte
        position = self.read_attempts - 1 if index is None else index
        if position >= len(self.line_sensor_sequence):
            position = len(self.line_sensor_sequence) - 1
        return self.line_sensor_sequence[max(0, position)]

    def close(self) -> None:
        self.closed = True

    # -- inspection helpers -------------------------------------------------

    @property
    def write_count(self) -> int:
        return len(self.writes)

    @property
    def payloads(self) -> list[tuple[int, ...]]:
        return [payload for _address, _register, payload in self.writes]

    @property
    def motor_payloads(self) -> list[tuple[int, ...]]:
        """Payloads addressed to the motor register."""
        from .registers import REG_MOTOR

        return [payload for _a, register, payload in self.writes if register == REG_MOTOR]

    def reset(self) -> None:
        self.writes.clear()
        self.reads.clear()
        self.write_attempts = 0
        self.read_attempts = 0


class MockLineSensorScript:
    """Sensor byte that can switch to a new pattern after N more reads.

    Used by mock sessions so a switch manoeuvre sees the source edge first and
    then the opposite edge, exactly as a real crossing would.
    """

    def __init__(self, byte: int = 0xFF) -> None:
        self.byte = int(byte)
        self._pending: tuple[int, int] | None = None
        self.read_count = 0

    def set_byte(self, byte: int) -> None:
        self.byte = int(byte)
        self._pending = None

    def schedule_byte(self, byte: int, *, after_reads: int) -> None:
        self._pending = (int(byte), max(0, int(after_reads)))

    def __call__(self) -> int:
        self.read_count += 1
        if self._pending is not None:
            byte, remaining = self._pending
            if remaining <= 0:
                self.byte = byte
                self._pending = None
            else:
                self._pending = (byte, remaining - 1)
        return self.byte


def build_mock_robot(
    config: RobotConfig | None = None,
    *,
    watchdog: bool = True,
    stop_check=None,
    telemetry=None,
):
    """Return ``(robot, transport)`` backed by :class:`MockI2CTransport`."""
    from ..robot import Robot

    transport = MockI2CTransport()
    return (
        Robot(
            transport,
            config,
            watchdog=watchdog,
            stop_check=stop_check,
            telemetry=telemetry,
        ),
        transport,
    )


__all__ = ["MockI2CTransport", "MockLineSensorScript", "build_mock_robot"]
