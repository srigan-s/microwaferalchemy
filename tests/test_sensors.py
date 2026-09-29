"""Line sensor decoding: polarity, channel mapping, and malformed frames."""

from __future__ import annotations

import pytest

from waferbot import (
    I2CError,
    MockI2CTransport,
    Robot,
    SensorConfig,
    SensorError,
    decode_line_byte,
)
from waferbot.hardware.registers import REG_LINE_SENSOR


def test_all_white_byte_normalises_to_white():
    reading = decode_line_byte(0xFF, timestamp=1.0)
    assert reading.raw == (1, 1, 1, 1)
    assert reading.normalized == (0, 0, 0, 0)
    assert reading.black_mask == 0
    assert reading.timestamp == 1.0


def test_all_black_byte_normalises_to_black():
    reading = decode_line_byte(0x00)
    assert reading.raw == (0, 0, 0, 0)
    assert reading.normalized == (1, 1, 1, 1)
    assert reading.black_mask == 0b1111


def test_channel_mapping_matches_vendor_left_to_right_order():
    # Vendor evidence maps bit 3 to S2, bit 2 to S1, bit 1 to S3, bit 0 to S4.
    config = SensorConfig()
    assert config.bit_for_channel == (2, 3, 1, 0)

    reading = decode_line_byte(0b0000_1000, config)
    # Only S2 (bit 3) is white; the other three channels see black.
    assert reading.raw == (0, 1, 0, 0)
    assert reading.normalized == (1, 0, 1, 1)


def test_black_polarity_is_configurable():
    inverted = SensorConfig(black_is_raw_zero=False)
    reading = decode_line_byte(0x00, inverted)
    assert reading.raw == (0, 0, 0, 0)
    assert reading.normalized == (0, 0, 0, 0)


@pytest.mark.parametrize(
    ("raw_byte", "expected_normalized"),
    [
        # S2 is bit 3, S3 is bit 1, and a raw 0 means black.
        (0b0000_0010, (1, 1, 0, 1)),  # S2 black, S3 white -> BLACK_LEFT
        (0b0000_1000, (1, 0, 1, 1)),  # S2 white, S3 black -> BLACK_RIGHT
        (0b0000_0000, (1, 1, 1, 1)),  # both middle black -> ambiguous BOTH_BLACK
        (0b0000_1010, (1, 0, 0, 1)),  # both middle white -> ambiguous BOTH_WHITE
    ],
)
def test_middle_channel_patterns(raw_byte, expected_normalized):
    assert decode_line_byte(raw_byte).normalized == expected_normalized


def test_decode_rejects_invalid_values():
    with pytest.raises(SensorError):
        decode_line_byte(-1)
    with pytest.raises(SensorError):
        decode_line_byte(256)
    with pytest.raises(SensorError):
        decode_line_byte("ff")  # type: ignore[arg-type]


def test_sensor_read_uses_documented_register(robot: Robot, transport: MockI2CTransport):
    transport.line_sensor_byte = 0b0000_0100
    reading = robot.read_line_sensors()
    assert transport.reads == [(0x2B, REG_LINE_SENSOR, 1)]
    assert reading.raw_byte == 0b0000_0100


def test_malformed_frame_length_is_a_sensor_error(
    robot: Robot, transport: MockI2CTransport
) -> None:
    transport.line_sensor_length = 3
    with pytest.raises(SensorError):
        robot.read_line_sensors()


def test_empty_frame_is_a_sensor_error(robot: Robot, transport: MockI2CTransport) -> None:
    transport.line_sensor_length = 0
    with pytest.raises(SensorError):
        robot.read_line_sensors()


def test_frame_byte_out_of_range_is_a_sensor_error(transport: MockI2CTransport) -> None:
    transport.line_sensor_byte = 300
    from waferbot import LineSensorArray

    with pytest.raises(SensorError):
        LineSensorArray(transport).read()


def test_read_error_is_i2c_error_not_defaulted(transport: MockI2CTransport) -> None:
    from waferbot import LineSensorArray

    transport.read_error = OSError("nack")
    with pytest.raises(I2CError) as excinfo:
        LineSensorArray(transport).read()
    assert isinstance(excinfo.value.__cause__, OSError)


def test_reading_timestamp_uses_injected_clock(robot: Robot, clock) -> None:
    reading = robot.read_line_sensors()
    assert reading.timestamp == clock.now
