"""Motor driver for the Yahboom Raspbot V2 four-wheel mecanum base.

Protocol (verified against vendored source, see ``registers.py``):

* one ``write_i2c_block_data`` per wheel, address ``0x2B``, register ``0x01``
* payload ``[motor_id, direction, speed]`` with ``direction`` 0 forward and
  1 backward, and ``speed`` the magnitude (0..255)

Two behaviours are deliberate departures from the vendored driver:

1. I2C exceptions are never printed and swallowed. They surface as
   :class:`~waferbot.errors.I2CError`.
2. If a multi-wheel command fails part-way through, every wheel is sent a stop
   block before the error is re-raised, so a partial command cannot leave the
   chassis driving on stale speed values.
"""

from __future__ import annotations

from collections.abc import Sequence

from ..errors import I2CError, MotorCommandError
from .bus import BusLock
from .registers import (
    I2C_ADDRESS_DEFAULT,
    MOTOR_ID_COUNT,
    REG_MOTOR,
    motor_block,
    stop_block,
)
from .transport import I2CTransport


class MotorDriver:
    """Strict, injectable motor driver. Construction never moves the robot."""

    def __init__(
        self,
        transport: I2CTransport,
        *,
        address: int = I2C_ADDRESS_DEFAULT,
        register: int = REG_MOTOR,
        lock: BusLock | None = None,
        invert: Sequence[bool] | None = None,
    ) -> None:
        if not isinstance(address, int) or not 0x03 <= address <= 0x77:
            raise ValueError(f"invalid motor I2C address: {address!r}")
        if not isinstance(register, int) or not 0x00 <= register <= 0xFF:
            raise ValueError(f"invalid motor register: {register!r}")
        self._transport = transport
        self._address = address
        self._register = register
        self._lock = lock or BusLock()
        flipped = tuple(invert) if invert is not None else (False, False, False, False)
        if len(flipped) != MOTOR_ID_COUNT or not all(
            isinstance(flag, bool) for flag in flipped
        ):
            raise ValueError("invert must be four booleans, one per wheel id")
        self._invert = flipped
        self._last_command: tuple[int, ...] | None = None

    @property
    def address(self) -> int:
        return self._address

    @property
    def register(self) -> int:
        return self._register

    @property
    def last_command(self) -> tuple[int, ...] | None:
        """Most recent per-wheel signed speeds, or ``None`` before any command."""
        return self._last_command

    @property
    def invert(self) -> tuple[bool, bool, bool, bool]:
        return self._invert  # type: ignore[return-value]

    def set_speeds(self, speeds: Sequence[int]) -> None:
        """Drive all four wheels.

        ``speeds`` holds signed PWM counts in motor-id order (0=L1, 1=L2,
        2=R1, 3=R2). On the first failure the remaining wheels are still sent a
        stop block and :class:`MotorCommandError` is raised.
        """
        if len(speeds) != MOTOR_ID_COUNT:
            raise ValueError(
                f"expected {MOTOR_ID_COUNT} wheel speeds, got {len(speeds)}"
            )
        for value in speeds:
            # Fractional or boolean counts are caller bugs. Coercing with int()
            # would silently drive a wheel the caller did not ask for.
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"wheel speeds must be int counts, got {value!r}")
        # Polarity calibration is applied at the wire level; ``last_command``
        # keeps the values the controller asked for.
        values = tuple(
            -speeds[i] if self._invert[i] else speeds[i]
            for i in range(MOTOR_ID_COUNT)
        )
        blocks = [motor_block(i, values[i]) for i in range(MOTOR_ID_COUNT)]

        failure: I2CError | None = None
        failed_index: int | None = None
        with self._lock:
            for index, block in enumerate(blocks):
                try:
                    self._transport.write_block(
                        self._address, self._register, block
                    )
                except I2CError as exc:
                    failure = exc
                    failed_index = index
                    break
        if failure is not None:
            self._last_command = None
            self._best_effort_stop()
            raise MotorCommandError(
                f"motor {failed_index} command failed; all wheels were sent a "
                f"stop block: {failure}"
            ) from failure
        self._last_command = tuple(int(speed) for speed in speeds)

    def stop_all(self) -> None:
        """Stop every wheel, attempting all four even if some writes fail.

        The first :class:`I2CError` is re-raised after every wheel has been
        attempted, so callers always learn that at least one stop failed.
        """
        self._last_command = None
        self._best_effort_stop(raise_on_failure=True)

    # -- internals ----------------------------------------------------------

    def _best_effort_stop(self, *, raise_on_failure: bool = False) -> None:
        first_error: I2CError | None = None
        with self._lock:
            for motor_id in range(MOTOR_ID_COUNT):
                try:
                    self._transport.write_block(
                        self._address, self._register, stop_block(motor_id)
                    )
                except I2CError as exc:
                    if first_error is None:
                        first_error = exc
        if first_error is not None and raise_on_failure:
            raise first_error


__all__ = ["MotorDriver"]
