"""Command builders for the Raspbot V2 I2C protocol.

The constants live in :mod:`waferbot.protocol` (a leaf module) and are
re-exported here so hardware code has a single import site. The builders encode
the vendored ``Ctrl_Muto`` / ``Ctrl_Car`` payload rules.
"""

from __future__ import annotations

from collections.abc import Sequence

from ..protocol import (
    I2C_ADDRESS_DEFAULT,
    I2C_BUS_DEFAULT,
    LINE_SENSOR_FRAME_BYTES,
    MOTOR_DIR_BACKWARD,
    MOTOR_DIR_FORWARD,
    MOTOR_ID_COUNT,
    MOTOR_SPEED_MAX,
    MOTOR_SPEED_MIN,
    REG_BEEP_SWITCH,
    REG_IR_REMOTE,
    REG_IR_SWITCH,
    REG_LED_ALL,
    REG_LED_ALONE,
    REG_LED_BRIGHTNESS_ALL,
    REG_LED_BRIGHTNESS_ALONE,
    REG_LINE_SENSOR,
    REG_MOTOR,
    REG_SERVO,
    REG_ULTRASONIC_HIGH,
    REG_ULTRASONIC_LOW,
    REG_ULTRASONIC_SWITCH,
)


def motor_block(motor_id: int, signed_speed: int) -> list[int]:
    """Build the ``Ctrl_Muto`` payload for one wheel.

    Matches the vendored clamp range of ``-255..255`` and its sign convention
    (``-255 <= speed < 0`` means backward, otherwise forward).
    """
    _check_motor_id(motor_id)
    if isinstance(signed_speed, bool) or not isinstance(signed_speed, int):
        raise ValueError(f"motor speed must be an int count, got {signed_speed!r}")
    if not -MOTOR_SPEED_MAX <= signed_speed <= MOTOR_SPEED_MAX:
        # Ctrl_Muto clamps beyond +-255; clamping here would mask a caller bug,
        # so the adapter refuses the command instead of silently changing it.
        raise ValueError(
            f"motor speed {signed_speed} outside supported range "
            f"-{MOTOR_SPEED_MAX}..{MOTOR_SPEED_MAX}"
        )
    direction = MOTOR_DIR_BACKWARD if signed_speed < 0 else MOTOR_DIR_FORWARD
    return [motor_id, direction, abs(signed_speed)]


def stop_block(motor_id: int) -> list[int]:
    """Stop payload, identical to the vendored ``stop_robot()`` bytes."""
    return [_check_motor_id(motor_id), MOTOR_DIR_FORWARD, 0]


def motor_blocks(speeds: Sequence[int]) -> list[list[int]]:
    """Build stop/run payloads for all four wheels in motor-id order."""
    if len(speeds) != MOTOR_ID_COUNT:
        raise ValueError(
            f"expected {MOTOR_ID_COUNT} wheel speeds, got {len(speeds)}"
        )
    return [motor_block(i, speeds[i]) for i in range(MOTOR_ID_COUNT)]


def _check_motor_id(motor_id: int) -> int:
    if motor_id not in range(MOTOR_ID_COUNT):
        raise ValueError(
            f"motor id {motor_id!r} outside 0..{MOTOR_ID_COUNT - 1}"
        )
    return motor_id


__all__ = [
    "I2C_ADDRESS_DEFAULT",
    "I2C_BUS_DEFAULT",
    "LINE_SENSOR_FRAME_BYTES",
    "MOTOR_DIR_BACKWARD",
    "MOTOR_DIR_FORWARD",
    "MOTOR_ID_COUNT",
    "MOTOR_SPEED_MAX",
    "MOTOR_SPEED_MIN",
    "REG_BEEP_SWITCH",
    "REG_IR_REMOTE",
    "REG_IR_SWITCH",
    "REG_LED_ALL",
    "REG_LED_ALONE",
    "REG_LED_BRIGHTNESS_ALL",
    "REG_LED_BRIGHTNESS_ALONE",
    "REG_LINE_SENSOR",
    "REG_MOTOR",
    "REG_SERVO",
    "REG_ULTRASONIC_HIGH",
    "REG_ULTRASONIC_LOW",
    "REG_ULTRASONIC_SWITCH",
    "motor_block",
    "motor_blocks",
    "stop_block",
]
