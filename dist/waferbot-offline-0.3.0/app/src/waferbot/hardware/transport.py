"""I2C transport abstractions.

``I2CTransport`` is the injectable seam: production code uses
``Smbus2Transport`` (which imports ``smbus2`` lazily so the package stays
importable on a developer machine), and tests inject ``MockI2CTransport``.

Failures are translated into :class:`~waferbot.errors.I2CError` with the
original exception chained. Nothing is printed and nothing is swallowed.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Protocol, runtime_checkable

from ..errors import I2CError


@runtime_checkable
class I2CTransport(Protocol):
    """Minimal SMBus surface used by this package."""

    def write_block(
        self, address: int, register: int, data: Sequence[int]
    ) -> None:
        """Write ``data`` to ``register`` on ``address`` (``write_i2c_block_data``)."""

    def read_block(self, address: int, register: int, length: int) -> list[int]:
        """Read ``length`` bytes from ``register`` on ``address``."""

    def close(self) -> None:
        """Release the bus. Safe to call more than once."""


class Smbus2Transport:
    """Real transport backed by ``smbus2`` on a Raspberry Pi.

    The smbus2 import happens inside :meth:`__init__`, so importing this module
    (and the rest of :mod:`waferbot`) never requires smbus2 and never touches
    hardware. Constructing the transport opens the bus; it does not move the
    robot.
    """

    def __init__(self, bus: int = 1) -> None:
        if not isinstance(bus, int) or bus < 0:
            raise I2CError(f"invalid I2C bus number: {bus!r}")
        self.bus_number = bus
        self._bus = self._open(bus)

    @staticmethod
    def _open(bus: int):
        try:
            from smbus2 import SMBus  # noqa: PLC0415 - deliberate lazy import
        except ImportError as exc:  # pragma: no cover - depends on host
            raise I2CError(
                "smbus2 is required for the physical I2C transport; install "
                'it with `pip install -e ".[hardware]"` on the Raspberry Pi'
            ) from exc
        try:
            return SMBus(bus)
        except Exception as exc:  # pragma: no cover - depends on host
            raise I2CError(f"cannot open I2C bus {bus}: {exc}") from exc

    def write_block(
        self, address: int, register: int, data: Sequence[int]
    ) -> None:
        try:
            self._bus.write_i2c_block_data(address, register, list(data))
        except Exception as exc:
            raise I2CError(
                f"I2C write failed (addr=0x{address:02X} reg=0x{register:02X} "
                f"data={list(data)}): {exc}"
            ) from exc

    def read_block(self, address: int, register: int, length: int) -> list[int]:
        try:
            values = self._bus.read_i2c_block_data(address, register, length)
        except Exception as exc:
            raise I2CError(
                f"I2C read failed (addr=0x{address:02X} reg=0x{register:02X} "
                f"len={length}): {exc}"
            ) from exc
        return list(values)

    def close(self) -> None:
        bus = getattr(self, "_bus", None)
        if bus is None:
            return
        try:
            bus.close()
        except Exception as exc:
            raise I2CError(f"failed to close I2C bus {self.bus_number}: {exc}") from exc
        finally:
            self._bus = None

    def __enter__(self) -> "Smbus2Transport":
        return self

    def __exit__(self, *_exc_info) -> None:
        self.close()


__all__ = ["I2CTransport", "Smbus2Transport"]

