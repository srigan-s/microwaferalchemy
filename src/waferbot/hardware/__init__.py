"""Hardware adapters: I2C transport, motor driver, line sensors, and mocks."""

from .mock import MockI2CTransport, MockLineSensorScript, build_mock_robot
from .motor_driver import MotorDriver
from .registers import (
    I2C_ADDRESS_DEFAULT,
    I2C_BUS_DEFAULT,
    MOTOR_ID_COUNT,
    REG_LINE_SENSOR,
    REG_MOTOR,
    motor_block,
    stop_block,
)
from .sensors import LineReading, LineSensorArray, decode_line_byte
from .transport import I2CTransport, Smbus2Transport

__all__ = [
    "I2C_ADDRESS_DEFAULT",
    "I2C_BUS_DEFAULT",
    "I2CTransport",
    "LineReading",
    "LineSensorArray",
    "MOTOR_ID_COUNT",
    "MockI2CTransport",
    "MockLineSensorScript",
    "MotorDriver",
    "REG_LINE_SENSOR",
    "REG_MOTOR",
    "Smbus2Transport",
    "build_mock_robot",
    "decode_line_byte",
    "motor_block",
    "stop_block",
]
