"""Raw I2C protocol constants for the Yahboom Raspbot V2 controller board.

This module is a leaf: it imports nothing from the rest of the package, so both
``waferbot.config`` and ``waferbot.hardware`` can depend on it without an
import cycle.

Every value traces to vendored source, not to guesswork:

* ``vendor/yahboom/project_demo/raspbot/Raspbot_Lib.py``
  - ``PI5Car_I2CADDR = 0x2B`` on I2C bus 1
  - ``Ctrl_Muto`` / ``Ctrl_Car`` write register ``0x01`` with
    ``[motor_id, direction, speed]`` via ``write_i2c_block_data``
  - ``read_data_array(0x0A, 1)`` returns the four-channel line sensor byte
  - ``0x1A`` / ``0x1B`` return the ultrasonic distance low/high bytes

Direction encoding: ``0`` = forward, ``1`` = backward; the speed byte is the
magnitude.
"""

from __future__ import annotations

I2C_ADDRESS_DEFAULT = 0x2B
I2C_BUS_DEFAULT = 1

# Write registers.
REG_MOTOR = 0x01
REG_SERVO = 0x02
REG_LED_ALL = 0x03
REG_LED_ALONE = 0x04
REG_IR_SWITCH = 0x05
REG_BEEP_SWITCH = 0x06
REG_ULTRASONIC_SWITCH = 0x07
REG_LED_BRIGHTNESS_ALL = 0x08
REG_LED_BRIGHTNESS_ALONE = 0x09
REG_IR_REMOTE = 0x0C

# Read registers.
REG_LINE_SENSOR = 0x0A
REG_ULTRASONIC_LOW = 0x1A
REG_ULTRASONIC_HIGH = 0x1B

# Motor frame layout: one block write per wheel, payload [id, direction, speed].
MOTOR_ID_COUNT = 4
MOTOR_DIR_FORWARD = 0
MOTOR_DIR_BACKWARD = 1
MOTOR_SPEED_MIN = 0
MOTOR_SPEED_MAX = 255

# The line sensor answers register 0x0A with a single packed byte.
LINE_SENSOR_FRAME_BYTES = 1

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
]
