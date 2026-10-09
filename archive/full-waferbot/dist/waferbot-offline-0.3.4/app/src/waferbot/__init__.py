"""MicroAlchemy physical robot package.

Phase 1 exposes the hardware abstraction layer only: a strict, injectable
I2C transport, a motor driver that speaks the protocol verified from the
vendored Yahboom sources, a four-channel line sensor reader, and a safety
controller that gates every motion command.

Importing this package performs no I2C traffic and no motion. ``smbus2`` is
imported lazily, so the package is importable on a development machine.
"""

from .config import MotorConfig, RobotConfig, SafetyConfig, SensorConfig
from .errors import (
    ConfigError,
    I2CError,
    MotorCommandError,
    SafetyError,
    SensorError,
    SpeedLimitError,
    WaferbotError,
)
from .hardware.motor_driver import MotorDriver
from .hardware.mock import MockI2CTransport, MockLineSensorScript, build_mock_robot
from .hardware.sensors import LineReading, LineSensorArray, decode_line_byte
from .hardware.transport import I2CTransport, Smbus2Transport
from .kinematics import DriveAction, wheel_speeds
from .protocol import I2C_ADDRESS_DEFAULT, I2C_BUS_DEFAULT
from .robot import Robot
from .safety import FaultCode, SafetyController, SafetyState
from .watchdog import MotionWatchdog

__all__ = [
    "ConfigError",
    "DriveAction",
    "FaultCode",
    "I2C_ADDRESS_DEFAULT",
    "I2C_BUS_DEFAULT",
    "I2CError",
    "I2CTransport",
    "LineReading",
    "LineSensorArray",
    "MockI2CTransport",
    "MockLineSensorScript",
    "MotionWatchdog",
    "MotorCommandError",
    "MotorConfig",
    "MotorDriver",
    "Robot",
    "RobotConfig",
    "SafetyConfig",
    "SafetyController",
    "SafetyError",
    "SafetyState",
    "SensorConfig",
    "SensorError",
    "Smbus2Transport",
    "SpeedLimitError",
    "WaferbotError",
    "build_mock_robot",
    "decode_line_byte",
    "wheel_speeds",
]

__version__ = "0.3.4"
