"""Packaged 128x32 SSD1306 OLED driver with Yahboom's public method names.

The vendored Yahboom notebooks call ``Yahboom_OLED(debug=False)``,
``init_oled_process()``, ``clear()``, ``add_line(text, row)`` and ``refresh()``.
Their driver file is not in this repository. This compatible implementation
uses the existing smbus2 dependency and never addresses the motor controller.

The default I2C bus/address (1, 0x3C) must be confirmed on the real Pi. All
transfers target the configured display address, never the Raspbot's 0x2B.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from ..errors import WaferbotError

WIDTH = 128
HEIGHT = 32
PAGES = HEIGHT // 8
DEFAULT_BUS = 1
DEFAULT_ADDRESS = 0x3C

# A compact 5x7 font. Lowercase glyphs used by the normal screen are included;
# other lowercase letters render with their uppercase glyphs.
_ROWS: dict[str, str] = {
    " ": "00000/00000/00000/00000/00000/00000/00000",
    "?": "01110/10001/00001/00010/00100/00000/00100",
    ":": "00000/00100/00100/00000/00100/00100/00000",
    "-": "00000/00000/00000/11111/00000/00000/00000",
    "0": "01110/10001/10011/10101/11001/10001/01110",
    "1": "00100/01100/00100/00100/00100/00100/01110",
    "2": "01110/10001/00001/00010/00100/01000/11111",
    "3": "11110/00001/00001/01110/00001/00001/11110",
    "4": "00010/00110/01010/10010/11111/00010/00010",
    "5": "11111/10000/10000/11110/00001/00001/11110",
    "6": "01110/10000/10000/11110/10001/10001/01110",
    "7": "11111/00001/00010/00100/01000/01000/01000",
    "8": "01110/10001/10001/01110/10001/10001/01110",
    "9": "01110/10001/10001/01111/00001/00001/01110",
    "A": "01110/10001/10001/11111/10001/10001/10001",
    "B": "11110/10001/10001/11110/10001/10001/11110",
    "C": "01111/10000/10000/10000/10000/10000/01111",
    "D": "11110/10001/10001/10001/10001/10001/11110",
    "E": "11111/10000/10000/11110/10000/10000/11111",
    "F": "11111/10000/10000/11110/10000/10000/10000",
    "G": "01111/10000/10000/10111/10001/10001/01110",
    "H": "10001/10001/10001/11111/10001/10001/10001",
    "I": "01110/00100/00100/00100/00100/00100/01110",
    "J": "00111/00010/00010/00010/10010/10010/01100",
    "K": "10001/10010/10100/11000/10100/10010/10001",
    "L": "10000/10000/10000/10000/10000/10000/11111",
    "M": "10001/11011/10101/10101/10001/10001/10001",
    "N": "10001/11001/10101/10011/10001/10001/10001",
    "O": "01110/10001/10001/10001/10001/10001/01110",
    "P": "11110/10001/10001/11110/10000/10000/10000",
    "Q": "01110/10001/10001/10001/10101/10010/01101",
    "R": "11110/10001/10001/11110/10100/10010/10001",
    "S": "01111/10000/10000/01110/00001/00001/11110",
    "T": "11111/00100/00100/00100/00100/00100/00100",
    "U": "10001/10001/10001/10001/10001/10001/01110",
    "V": "10001/10001/10001/10001/10001/01010/00100",
    "W": "10001/10001/10001/10101/10101/10101/01010",
    "X": "10001/10001/01010/00100/01010/10001/10001",
    "Y": "10001/10001/01010/00100/00100/00100/00100",
    "Z": "11111/00001/00010/00100/01000/10000/11111",
    "a": "00000/00000/01110/00001/01111/10001/01111",
    "b": "10000/10000/10110/11001/10001/10001/11110",
    "d": "00001/00001/01101/10011/10001/10001/01111",
    "e": "00000/00000/01110/10001/11111/10000/01110",
    "f": "00110/01001/01000/11100/01000/01000/01000",
    "o": "00000/00000/01110/10001/10001/10001/01110",
    "r": "00000/00000/10110/11001/10000/10000/10000",
    "t": "01000/01000/11100/01000/01000/01001/00110",
    "y": "00000/00000/10001/10001/01111/00001/01110",
}


def _glyph(ch: str) -> tuple[int, ...]:
    rows = _ROWS.get(ch, _ROWS.get(ch.upper(), _ROWS["?"])).split("/")
    return tuple(
        sum((row[column] == "1") << y for y, row in enumerate(rows))
        for column in range(5)
    )


class Yahboom_OLED:
    """Synchronous drop-in for the stock OLED calls used by Yahboom demos.

    ``init_oled_process`` keeps its stock name for compatibility but starts no
    background process. Only the caller's explicit ``refresh`` sends pixels.
    """

    def __init__(
        self,
        debug: bool = False,
        *,
        bus: int = DEFAULT_BUS,
        address: int = DEFAULT_ADDRESS,
        bus_factory: Callable[[int], Any] | None = None,
    ) -> None:
        if bus < 0 or not 0x03 <= address <= 0x77:
            raise ValueError("OLED bus or I2C address is out of range")
        self.debug = debug
        self.bus_number = bus
        self.address = address
        self._bus_factory = bus_factory
        self._bus: Any | None = None
        self._buffer = bytearray(WIDTH * PAGES)

    def init_oled_process(self) -> None:
        if self._bus is not None:
            return
        if self._bus_factory is None:
            try:
                from smbus2 import SMBus
            except ImportError as exc:
                raise WaferbotError("smbus2 is required for the packaged OLED driver") from exc
            self._bus_factory = SMBus
        try:
            self._bus = self._bus_factory(self.bus_number)
            # SSD1306 128x32 initialization; horizontal framebuffer addressing.
            for command in (
                0xAE, 0xD5, 0x80, 0xA8, 0x1F, 0xD3, 0x00, 0x40,
                0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x02,
                0x81, 0x8F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF,
            ):
                self._write(0x00, [command])
        except Exception as exc:
            self.close()
            raise WaferbotError(
                f"OLED initialization failed on I2C bus {self.bus_number} "
                f"at 0x{self.address:02X}: {exc}"
            ) from exc

    def _write(self, control: int, data: list[int] | bytes | bytearray) -> None:
        if self._bus is None:
            raise WaferbotError("OLED is not initialized")
        self._bus.write_i2c_block_data(self.address, control, list(data))

    def clear(self) -> None:
        self._buffer[:] = bytes(len(self._buffer))

    def add_line(self, text: str, row: int) -> None:
        if not 1 <= row <= PAGES:
            raise ValueError("OLED row must be 1..4")
        x = 0
        offset = (row - 1) * WIDTH
        for ch in str(text):
            if x + 5 > WIDTH:
                break
            self._buffer[offset + x : offset + x + 5] = bytes(_glyph(ch))
            x += 6

    def refresh(self) -> None:
        if self._bus is None:
            raise WaferbotError("OLED is not initialized")
        try:
            for command in (0x21, 0, WIDTH - 1, 0x22, 0, PAGES - 1):
                self._write(0x00, [command])
            for offset in range(0, len(self._buffer), 16):
                self._write(0x40, self._buffer[offset : offset + 16])
        except OSError as exc:
            raise WaferbotError(
                f"OLED refresh failed on I2C bus {self.bus_number} "
                f"at 0x{self.address:02X}: {exc}"
            ) from exc

    def close(self) -> None:
        if self._bus is not None:
            self._bus.close()
            self._bus = None


__all__ = ["Yahboom_OLED", "DEFAULT_BUS", "DEFAULT_ADDRESS"]
