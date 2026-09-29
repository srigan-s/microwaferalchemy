"""Exact I2C protocol assertions for the motor driver.

Every expected payload here is copied from the behaviour of
``vendor/yahboom/project_demo/raspbot/Raspbot_Lib.py`` (``Ctrl_Muto`` /
``Ctrl_Car``) and ``vendor/yahboom/project_demo/lib/McLumk_Wheel_Sports.py``.
"""

from __future__ import annotations

import pytest

from waferbot import (
    I2C_ADDRESS_DEFAULT,
    MockI2CTransport,
    Robot,
    RobotConfig,
    SpeedLimitError,
)
from waferbot.hardware.registers import (
    REG_MOTOR,
    motor_block,
    motor_blocks,
    stop_block,
)


def test_motor_block_matches_ctrl_muto_encoding():
    assert motor_block(0, 150) == [0, 0, 150]
    assert motor_block(3, -50) == [3, 1, 50]
    assert motor_block(2, 0) == [2, 0, 0]
    assert stop_block(1) == [1, 0, 0]


def test_motor_block_rejects_out_of_protocol_range():
    with pytest.raises(ValueError):
        motor_block(0, 256)
    with pytest.raises(ValueError):
        motor_block(0, -256)
    with pytest.raises(ValueError):
        motor_block(4, 10)


def test_motor_blocks_requires_four_wheels():
    with pytest.raises(ValueError):
        motor_blocks([10, 10, 10])
    assert motor_blocks([1, -1, 2, -2]) == [[0, 0, 1], [1, 1, 1], [2, 0, 2], [3, 1, 2]]


def test_construction_and_import_write_nothing(transport: MockI2CTransport) -> None:
    Robot(transport, RobotConfig())
    assert transport.writes == []
    assert transport.reads == []


def test_forward_writes_four_blocks_in_motor_id_order(
    robot: Robot, transport: MockI2CTransport
) -> None:
    robot.arm()
    robot.forward(60)

    assert transport.writes == [
        (I2C_ADDRESS_DEFAULT, REG_MOTOR, (0, 0, 60)),
        (I2C_ADDRESS_DEFAULT, REG_MOTOR, (1, 0, 60)),
        (I2C_ADDRESS_DEFAULT, REG_MOTOR, (2, 0, 60)),
        (I2C_ADDRESS_DEFAULT, REG_MOTOR, (3, 0, 60)),
    ]
    assert robot.last_command == (60, 60, 60, 60)


@pytest.mark.parametrize(
    ("method", "speed", "expected"),
    [
        ("backward", 60, [(0, 1, 60), (1, 1, 60), (2, 1, 60), (3, 1, 60)]),
        ("strafe_left", 60, [(0, 1, 60), (1, 0, 60), (2, 0, 60), (3, 1, 60)]),
        ("strafe_right", 60, [(0, 0, 60), (1, 1, 60), (2, 1, 60), (3, 0, 60)]),
        ("rotate_left", 60, [(0, 1, 60), (1, 1, 60), (2, 0, 60), (3, 0, 60)]),
        ("rotate_right", 60, [(0, 0, 60), (1, 0, 60), (2, 1, 60), (3, 1, 60)]),
    ],
)
def test_drive_primitives_match_vendor_mixing(
    robot: Robot, transport: MockI2CTransport, method: str, speed: int, expected
) -> None:
    robot.arm()
    getattr(robot, method)(speed)
    assert transport.payloads == expected


def test_rotation_and_strafe_are_distinct_patterns(
    robot: Robot, transport: MockI2CTransport
) -> None:
    robot.arm()
    robot.rotate_left(50)
    rotate_writes = list(transport.payloads)
    transport.reset()
    robot.strafe_left(50)
    strafe_writes = list(transport.payloads)
    assert rotate_writes != strafe_writes
    # Rotation is differential (both left wheels share a sign), strafe is not.
    assert rotate_writes[0][1:] == rotate_writes[1][1:]
    assert strafe_writes[0][1:] != strafe_writes[1][1:]


def test_stop_writes_zero_speed_for_every_wheel(
    robot: Robot, transport: MockI2CTransport
) -> None:
    robot.arm()
    robot.forward(40)
    transport.reset()
    robot.stop()
    assert transport.payloads == [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]
    assert robot.last_command is None


def test_speed_above_configured_limit_is_refused(
    transport: MockI2CTransport, clock
) -> None:
    robot = Robot(transport, RobotConfig().with_speed_limit(80), clock=clock)
    robot.arm()
    with pytest.raises(SpeedLimitError):
        robot.forward(81)
    assert transport.writes == []


@pytest.mark.parametrize("bad_speed", [-1, 255, 1.5, True, "50"])
def test_invalid_speed_values_are_refused(
    robot: Robot, transport: MockI2CTransport, bad_speed
) -> None:
    robot.arm()
    with pytest.raises(SpeedLimitError):
        robot.forward(bad_speed)
    assert transport.writes == []
