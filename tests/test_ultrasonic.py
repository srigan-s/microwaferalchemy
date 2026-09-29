"""Ultrasonic distance reads and the obstacle monitor hook."""

from __future__ import annotations

import time

import pytest

from waferbot import I2CError, MockI2CTransport
from waferbot.hardware.ultrasonic import (
    ULTRASONIC_MAX_VALID_MM,
    ObstacleMonitor,
    UltrasonicSensor,
)
from waferbot.protocol import (
    REG_ULTRASONIC_HIGH,
    REG_ULTRASONIC_LOW,
    REG_ULTRASONIC_SWITCH,
)


class DistanceTransport(MockI2CTransport):
    """Mock transport that answers ultrasonic register reads."""

    def __init__(self, distance_mm: int) -> None:
        super().__init__()
        self.distance_mm = distance_mm

    def read_block(self, address: int, register: int, length: int) -> list[int]:
        self.reads.append((address, register, length))
        if self.read_error is not None:
            raise I2CError("mock read failed") from self.read_error
        if register == REG_ULTRASONIC_HIGH:
            return [(self.distance_mm >> 8) & 0xFF]
        if register == REG_ULTRASONIC_LOW:
            return [self.distance_mm & 0xFF]
        return [self.line_sensor_byte] * max(0, self.line_sensor_length)


def test_distance_is_combined_from_high_and_low_bytes():
    transport = DistanceTransport(1234)
    sensor = UltrasonicSensor(transport)
    assert sensor.read_distance_mm() == 1234
    assert [register for _a, register, _l in transport.reads] == [
        REG_ULTRASONIC_HIGH,
        REG_ULTRASONIC_LOW,
    ]


@pytest.mark.parametrize("distance", [0, ULTRASONIC_MAX_VALID_MM + 1])
def test_out_of_range_frames_are_reported_as_no_echo(distance):
    sensor = UltrasonicSensor(DistanceTransport(distance))
    assert sensor.read_distance_mm() is None


def test_enable_writes_the_documented_register():
    transport = MockI2CTransport()
    sensor = UltrasonicSensor(transport)
    sensor.enable(True)
    sensor.enable(False)
    assert transport.payloads == [(1,), (0,)]
    assert all(
        register == REG_ULTRASONIC_SWITCH for _a, register, _p in transport.writes
    )


def test_read_failure_propagates():
    transport = DistanceTransport(500)
    transport.read_error = OSError("nack")
    sensor = UltrasonicSensor(transport)
    with pytest.raises(I2CError):
        sensor.read_distance_mm()


def test_obstacle_monitor_triggers_below_threshold():
    transport = DistanceTransport(300)
    sensor = UltrasonicSensor(transport)
    seen: list[int] = []
    monitor = ObstacleMonitor(sensor, seen.append, threshold_mm=200)
    assert monitor.poll_once() == 300
    assert seen == []

    transport.distance_mm = 150
    assert monitor.poll_once() == 150
    assert seen == [150]
    assert monitor.trigger_count == 1


def test_obstacle_monitor_thread_start_and_stop():
    transport = DistanceTransport(100)
    sensor = UltrasonicSensor(transport)
    seen: list[int] = []
    monitor = ObstacleMonitor(
        sensor, seen.append, threshold_mm=200, interval_s=0.005
    )
    monitor.start()
    deadline = time.monotonic() + 2.0
    while not seen and time.monotonic() < deadline:
        time.sleep(0.005)
    monitor.stop()
    assert seen, "monitor never reported the obstacle"
    assert monitor.running is False
    # Disabling ranging is attempted on stop, so the last write is a switch-off.
    assert transport.payloads[-1] == (0,)


def test_obstacle_monitor_validates_configuration():
    sensor = UltrasonicSensor(DistanceTransport(100))
    with pytest.raises(ValueError):
        ObstacleMonitor(sensor, lambda _d: None, threshold_mm=0)
    with pytest.raises(ValueError):
        ObstacleMonitor(sensor, lambda _d: None, threshold_mm=10, interval_s=0)

