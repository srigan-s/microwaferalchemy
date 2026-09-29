"""Arming, emergency stop latching, faults, and the motion watchdog."""

from __future__ import annotations

import pytest

from waferbot import (
    ConfigError,
    FaultCode,
    MockI2CTransport,
    Robot,
    RobotConfig,
    SafetyConfig,
    SafetyError,
    SafetyState,
)


def test_motion_is_refused_until_armed(robot: Robot, transport: MockI2CTransport) -> None:
    with pytest.raises(SafetyError):
        robot.forward(40)
    with pytest.raises(SafetyError):
        robot.rotate_left(40)
    assert transport.writes == []
    assert robot.armed is False


def test_arm_then_drive_then_disarm(robot: Robot, transport: MockI2CTransport) -> None:
    robot.arm()
    assert robot.armed is True
    robot.forward(40)
    assert transport.write_count == 4
    robot.disarm()
    with pytest.raises(SafetyError):
        robot.forward(40)
    # Disarm stops the wheels instead of only clearing the deadline.
    assert transport.payloads[-4:] == [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]
    assert transport.write_count == 8


def test_sensor_reads_are_allowed_while_disarmed(robot: Robot) -> None:
    reading = robot.read_line_sensors()
    assert reading.raw_byte == 0xFF


def test_stop_is_allowed_while_disarmed(robot: Robot, transport: MockI2CTransport) -> None:
    robot.stop()
    assert transport.payloads == [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]


def test_emergency_stop_latches_and_blocks_motion(
    robot: Robot, transport: MockI2CTransport
) -> None:
    robot.arm()
    robot.forward(50)
    transport.reset()

    record = robot.emergency_stop(FaultCode.OBSTACLE_DETECTED, "object 10cm ahead")

    assert record.code is FaultCode.OBSTACLE_DETECTED
    assert robot.state is SafetyState.EMERGENCY_STOP
    assert robot.emergency_stop_latched is True
    assert robot.armed is False
    assert transport.payloads == [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]

    transport.reset()
    with pytest.raises(SafetyError):
        robot.forward(50)
    with pytest.raises(SafetyError):
        robot.arm()
    assert transport.writes == []


def test_clearing_emergency_stop_requires_confirmation(robot: Robot) -> None:
    robot.arm()
    robot.emergency_stop()
    with pytest.raises(SafetyError):
        robot.clear_emergency_stop()
    assert robot.emergency_stop_latched is True

    robot.clear_emergency_stop(confirm=True)
    assert robot.emergency_stop_latched is False
    assert robot.armed is False
    with pytest.raises(SafetyError):
        robot.forward(40)
    robot.arm()
    robot.forward(40)


def test_fault_blocks_arming_until_cleared(robot: Robot, transport: MockI2CTransport) -> None:
    robot.arm()
    transport.reset()
    record = robot.raise_fault(FaultCode.LINE_LOST, "no tape under any sensor")
    assert record.code is FaultCode.LINE_LOST
    assert robot.state is SafetyState.FAULT
    assert transport.payloads == [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]

    with pytest.raises(SafetyError):
        robot.arm()
    with pytest.raises(SafetyError):
        robot.forward(40)

    robot.clear_fault()
    robot.arm()
    transport.reset()
    robot.forward(30)
    assert transport.write_count == 4


def test_clear_fault_without_fault_is_refused(robot: Robot) -> None:
    with pytest.raises(SafetyError):
        robot.clear_fault()


def test_watchdog_stops_motors_after_deadline(
    robot: Robot, transport: MockI2CTransport, clock
) -> None:
    robot.arm()
    robot.forward(50)
    transport.reset()

    assert robot.tick() is False
    clock.advance(robot.safety.motion_timeout_s + 0.001)
    assert robot.tick() is True
    assert transport.payloads == [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]


def test_motion_command_refreshes_the_deadline(
    robot: Robot, transport: MockI2CTransport, clock
) -> None:
    robot.arm()
    robot.forward(50)
    for _ in range(5):
        clock.advance(robot.safety.motion_timeout_s / 2)
        robot.forward(50)
        assert robot.tick() is False
    assert transport.write_count == 24


def test_stop_clears_pending_deadline(robot: Robot, transport: MockI2CTransport, clock) -> None:
    robot.arm()
    robot.forward(50)
    robot.stop()
    transport.reset()
    clock.advance(10)
    assert robot.tick() is False
    assert transport.writes == []


def test_motion_timeout_must_be_positive(transport: MockI2CTransport, clock) -> None:
    bad = RobotConfig(safety=SafetyConfig(motion_timeout_s=0.0))
    with pytest.raises(ConfigError):
        Robot(transport, bad, clock=clock)


def test_state_transitions_are_explicit(robot: Robot) -> None:
    robot.set_state(SafetyState.FOLLOWING)
    assert robot.state is SafetyState.FOLLOWING
    robot.disarm()
    assert robot.state is SafetyState.IDLE
