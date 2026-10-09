"""Fault behaviour: no swallowed I2C errors and best-effort stops on failure."""

from __future__ import annotations

import pytest

from waferbot import (
    FaultCode,
    I2CError,
    MockI2CTransport,
    MotorCommandError,
    Robot,
    RobotConfig,
)


def test_partial_command_failure_stops_every_wheel_and_raises(
    robot: Robot, transport: MockI2CTransport
) -> None:
    # The third motor write (motor id 2) fails after motors 0 and 1 were set.
    transport.fail_write_indices = {2}
    robot.arm()

    with pytest.raises(MotorCommandError) as excinfo:
        robot.forward(60)

    assert isinstance(excinfo.value, I2CError)
    assert isinstance(excinfo.value.__cause__, I2CError)
    assert isinstance(excinfo.value.__cause__.__cause__, OSError)
    # No further speed writes after the failure: exactly two ran, then the
    # driver's best-effort stop for all four wheels, then the robot's second
    # stop attempt while latching MOTOR_COMMUNICATION_FAILURE.
    assert transport.payloads == [
        (0, 0, 60),
        (1, 0, 60),
        (0, 0, 0),
        (1, 0, 0),
        (2, 0, 0),
        (3, 0, 0),
        (0, 0, 0),
        (1, 0, 0),
        (2, 0, 0),
        (3, 0, 0),
    ]
    assert robot.last_command is None


def test_stop_all_attempts_every_wheel_even_when_all_writes_fail(
    robot: Robot, transport: MockI2CTransport
) -> None:
    transport.fail_write_indices = {0, 1, 2, 3}
    with pytest.raises(I2CError):
        robot.stop()
    assert transport.writes == []
    assert transport.write_count == 0


def test_stop_all_reports_failure_after_attempting_all_wheels(
    robot: Robot, transport: MockI2CTransport
) -> None:
    # Only the last wheel fails: the first three stops must still be written.
    transport.fail_write_indices = {3}
    with pytest.raises(I2CError) as excinfo:
        robot.stop()
    assert isinstance(excinfo.value.__cause__, OSError)
    assert transport.payloads == [(0, 0, 0), (1, 0, 0), (2, 0, 0)]


def test_sensor_read_failure_propagates(transport: MockI2CTransport, robot: Robot) -> None:
    transport.read_error = OSError("bus wedged")
    with pytest.raises(I2CError) as excinfo:
        robot.read_line_sensors()
    assert isinstance(excinfo.value.__cause__, OSError)
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.SENSOR_FAILURE


def test_motor_command_error_is_an_i2c_error_subclass() -> None:
    assert issubclass(MotorCommandError, I2CError)


def test_failed_command_leaves_robot_able_to_stop_again(
    robot: Robot, transport: MockI2CTransport
) -> None:
    transport.fail_write_indices = {1}
    robot.arm()
    with pytest.raises(MotorCommandError):
        robot.forward(30)
    transport.fail_write_indices = set()
    transport.reset()
    robot.stop()
    assert transport.payloads == [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]
