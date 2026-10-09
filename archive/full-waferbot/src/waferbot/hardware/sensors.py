"""Four-channel infrared line sensor reader.

Protocol (verified against vendored source, see ``registers.py``):

* ``read_i2c_block_data(0x2B, 0x0A, 1)`` returns one byte holding the four
  channel bits.
* Channel polarity: the vendored line-follower comments state that ``0`` means
  the black line is detected and ``1`` means white, e.g.
  ``# 都是黑色, 加速前进 All black`` for an all-zero byte and
  ``# 右锐角...0表示检测到黑线 ... 0 means black line is detected``.

Normalisation required by the project follows from that: ``BLACK = 1`` and
``WHITE = 0`` after normalisation, so a raw ``0`` bit becomes a normalised
``1``. Both the polarity and the bit-to-channel order are configurable because
both need a one-time confirmation on the real chassis.

Reads raise :class:`~waferbot.errors.I2CError` on bus failure and
:class:`~waferbot.errors.SensorError` on a malformed frame. Nothing is
swallowed and no default value is invented.
"""

from __future__ import annotations

import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass

from ..config import SensorConfig
from ..errors import SensorError
from .bus import BusLock
from .registers import LINE_SENSOR_FRAME_BYTES
from .transport import I2CTransport


@dataclass(frozen=True)
class LineReading:
    """One decoded sensor frame.

    ``raw`` and ``normalized`` are ordered S1, S2, S3, S4 (left to right).
    ``S2``/``S3`` are the middle pair used for edge detection in phase 2.
    """

    raw_byte: int
    raw: tuple[int, int, int, int]
    normalized: tuple[int, int, int, int]
    timestamp: float

    @property
    def channels(self) -> tuple[int, int, int, int]:
        return self.normalized

    @property
    def black_mask(self) -> int:
        """Normalised channels packed as a nibble, S1 in bit 0."""
        mask = 0
        for index, value in enumerate(self.normalized):
            mask |= (value & 1) << index
        return mask

    def as_dict(self) -> dict[str, object]:
        return {
            "raw_byte": self.raw_byte,
            "raw": list(self.raw),
            "normalized": list(self.normalized),
            "timestamp": self.timestamp,
        }


def decode_line_byte(
    raw_byte: int,
    config: SensorConfig | None = None,
    *,
    timestamp: float | None = None,
) -> LineReading:
    """Decode a sensor byte into a :class:`LineReading` (pure function)."""
    config = config or SensorConfig()
    if isinstance(raw_byte, bool) or not isinstance(raw_byte, int):
        raise SensorError(f"line sensor byte must be an int, got {raw_byte!r}")
    if not 0x00 <= raw_byte <= 0xFF:
        raise SensorError(f"line sensor byte out of range: {raw_byte!r}")
    raw_channels = []
    normalized = []
    for bit in config.bit_for_channel:
        bit_value = (raw_byte >> bit) & 0x01
        raw_channels.append(bit_value)
        if config.black_is_raw_zero:
            normalized.append(0 if bit_value else 1)
        else:
            normalized.append(bit_value)
    return LineReading(
        raw_byte=raw_byte,
        raw=tuple(raw_channels),  # type: ignore[arg-type]
        normalized=tuple(normalized),  # type: ignore[arg-type]
        timestamp=time.monotonic() if timestamp is None else timestamp,
    )


class LineSensorArray:
    """Reads the four-channel line sensor over an injected transport."""

    def __init__(
        self,
        transport: I2CTransport,
        config: SensorConfig | None = None,
        *,
        clock: Callable[[], float] = time.monotonic,
        lock: BusLock | None = None,
    ) -> None:
        self._config = (config or SensorConfig())
        self._config.validate()
        self._transport = transport
        self._clock = clock
        self._lock = lock or BusLock()

    @property
    def config(self) -> SensorConfig:
        return self._config

    def read_raw(self) -> int:
        """Read the packed sensor byte, validating the frame length."""
        with self._lock:
            values: Sequence[int] = self._transport.read_block(
                self._config.address,
                self._config.register,
                LINE_SENSOR_FRAME_BYTES,
            )
        if len(values) != LINE_SENSOR_FRAME_BYTES:
            raise SensorError(
                "line sensor frame has "
                f"{len(values)} bytes, expected {LINE_SENSOR_FRAME_BYTES}"
            )
        value = values[0]
        if not isinstance(value, int) or isinstance(value, bool):
            raise SensorError(f"line sensor byte is not an int: {value!r}")
        if not 0x00 <= value <= 0xFF:
            raise SensorError(f"line sensor byte out of range: {value!r}")
        return value

    def read(self) -> LineReading:
        """Read and decode one frame. Bus errors propagate unchanged."""
        raw_byte = self.read_raw()
        return decode_line_byte(raw_byte, self._config, timestamp=self._clock())


__all__ = ["LineReading", "LineSensorArray", "decode_line_byte"]
