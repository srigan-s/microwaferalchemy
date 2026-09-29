"""Ultrasonic distance sensor and an obstacle monitor hook.

Protocol (vendored source, ``Raspbot_Lib.py``): enable register ``0x07``,
distance high byte at ``0x1B`` and low byte at ``0x1A``, combined as
``high << 8 | low`` in millimetres.

The monitor is how the safety layer learns about obstacles: when the measured
distance drops below the threshold it calls ``on_obstacle``, which the CLI wires
to ``Robot.emergency_stop(FaultCode.OBSTACLE_DETECTED, ...)``.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable

from ..errors import I2CError
from ..protocol import (
    I2C_ADDRESS_DEFAULT,
    REG_ULTRASONIC_HIGH,
    REG_ULTRASONIC_LOW,
    REG_ULTRASONIC_SWITCH,
)
from .bus import BusLock
from .transport import I2CTransport

#: Frames at or above this value are treated as "no echo" rather than a distance.
ULTRASONIC_MAX_VALID_MM = 4500


class UltrasonicSensor:
    """Reads the board's ultrasonic distance in millimetres."""

    def __init__(
        self,
        transport: I2CTransport,
        *,
        address: int = I2C_ADDRESS_DEFAULT,
        lock: BusLock | None = None,
    ) -> None:
        self._transport = transport
        self._address = address
        self._lock = lock or BusLock()

    def enable(self, state: bool) -> None:
        """Turn the ranging module on or off (vendored ``Ctrl_Ulatist_Switch``)."""
        with self._lock:
            self._transport.write_block(
                self._address, REG_ULTRASONIC_SWITCH, [1 if state else 0]
            )

    def read_distance_mm(self) -> int | None:
        """Return a millimetre distance, or ``None`` when the frame is not usable."""
        with self._lock:
            high = self._transport.read_block(self._address, REG_ULTRASONIC_HIGH, 1)
            low = self._transport.read_block(self._address, REG_ULTRASONIC_LOW, 1)
        if len(high) != 1 or len(low) != 1:
            raise I2CError("ultrasonic frame is not two bytes")
        distance = (int(high[0]) << 8) | int(low[0])
        if distance <= 0 or distance > ULTRASONIC_MAX_VALID_MM:
            return None
        return distance


class ObstacleMonitor:
    """Polls the ultrasonic sensor and reports obstacles below a threshold."""

    def __init__(
        self,
        sensor: UltrasonicSensor,
        on_obstacle: Callable[[int], None],
        *,
        threshold_mm: int,
        interval_s: float = 0.1,
        enable_ranging: bool = True,
    ) -> None:
        if threshold_mm <= 0:
            raise ValueError("threshold_mm must be positive")
        if interval_s <= 0:
            raise ValueError("interval_s must be positive")
        self.sensor = sensor
        self.on_obstacle = on_obstacle
        self.threshold_mm = int(threshold_mm)
        self.interval_s = float(interval_s)
        self.enable_ranging = enable_ranging
        self.last_distance_mm: int | None = None
        self.trigger_count = 0
        self.last_error: BaseException | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    @property
    def running(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    def poll_once(self) -> int | None:
        """One measurement, with the obstacle callback applied when needed."""
        distance = self.sensor.read_distance_mm()
        self.last_distance_mm = distance
        if distance is not None and distance < self.threshold_mm:
            self.trigger_count += 1
            self.on_obstacle(distance)
        return distance

    def start(self) -> None:
        if self.running:
            return
        if self.enable_ranging:
            self.sensor.enable(True)
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run, name="waferbot-obstacle", daemon=True
        )
        self._thread.start()

    def stop(self, *, join_timeout_s: float = 1.0) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(join_timeout_s)
        self._thread = None
        if self.enable_ranging:
            try:
                self.sensor.enable(False)
            except I2CError:
                # Ranging being left on is not a motion hazard; the wheel stop
                # is handled by the safety layer.
                pass

    def _run(self) -> None:
        while not self._stop_event.wait(self.interval_s):
            try:
                self.poll_once()
            except BaseException as exc:  # pragma: no cover - defensive
                self.last_error = exc


__all__ = ["ObstacleMonitor", "ULTRASONIC_MAX_VALID_MM", "UltrasonicSensor"]

