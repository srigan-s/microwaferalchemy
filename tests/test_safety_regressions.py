"""Regression tests for the phase 2 safety corrections.

These were written before the implementation changes they cover:

1. ``disarm()`` must stop the wheels, not just clear the deadline.
2. Motor command failures must latch ``MOTOR_COMMUNICATION_FAILURE``.
3. Sensor read failures must latch ``SENSOR_FAILURE``.
4. A failed stop must latch and keep the robot disarmed.
5. The independent watchdog must stop the wheels without ``tick()``.
6. An external stop request must refuse motion and stop.
7. Fractional/boolean wheel counts must be rejected, not coerced.
8. ``motor.bus`` and ``sensor.bus`` must not disagree.
"""

from __future__ import annotations

import threading
import time

import pytest

from waferbot import (
    ConfigError,
    FaultCode,
    I2CError,
    MockI2CTransport,
    MotorCommandError,
    MotorConfig,
    MotorDriver,
    Robot,
    RobotConfig,
    SafetyConfig,
    SafetyError,
    SensorConfig,
    SensorError,
    SpeedLimitError,
)

STOP_PAYLOADS = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]


def wait_until(predicate, timeout_s: float = 2.0, interval_s: float = 0.005) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return predicate()


# -- 1. disarm stops the wheels ----------------------------------------------


def test_disarm_stops_the_wheels(robot: Robot, transport: MockI2CTransport) -> None:
    robot.arm()
    robot.forward(40)
    transport.reset()

    robot.disarm()

    assert transport.payloads == STOP_PAYLOADS
    assert robot.armed is False


def test_disarm_failure_latches_and_keeps_robot_disarmed(
    robot: Robot, transport: MockI2CTransport
) -> None:
    robot.arm()
    robot.forward(40)
    transport.reset()
    transport.fail_write_indices = {0, 1, 2, 3}

    with pytest.raises(I2CError):
        robot.disarm()

    assert robot.armed is False
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.MOTOR_COMMUNICATION_FAILURE
    with pytest.raises(SafetyError):
        robot.arm()


# -- 2. motor command failure latches ----------------------------------------


def test_motor_failure_latches_fault_stops_and_preserves_cause(
    robot: Robot, transport: MockI2CTransport
) -> None:
    robot.arm()
    transport.fail_write_indices = {2}

    with pytest.raises(MotorCommandError) as excinfo:
        robot.forward(60)

    assert isinstance(excinfo.value.__cause__, I2CError)
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.MOTOR_COMMUNICATION_FAILURE
    assert robot.armed is False
    # The driver stops after its own partial write, then the robot stops again
    # while latching the fault: the last four payloads are always stops.
    assert transport.payloads[-4:] == STOP_PAYLOADS
    with pytest.raises(SafetyError):
        robot.forward(60)


def test_motor_failure_while_stopping_uses_typed_error(
    robot: Robot, transport: MockI2CTransport
) -> None:
    robot.arm()
    transport.fail_all_writes = True
    with pytest.raises(MotorCommandError) as excinfo:
        robot.forward(60)
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.MOTOR_COMMUNICATION_FAILURE
    assert "failed to stop the wheels" in robot.fault.message
    assert excinfo.value.__cause__ is not None


# -- 3. sensor failure latches ------------------------------------------------


def test_sensor_read_failure_latches_and_stops(
    robot: Robot, transport: MockI2CTransport
) -> None:
    robot.arm()
    robot.forward(40)
    transport.reset()
    transport.read_error = OSError("nack")

    with pytest.raises(I2CError) as excinfo:
        robot.read_line_sensors()

    assert isinstance(excinfo.value.__cause__, OSError)
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.SENSOR_FAILURE
    assert robot.armed is False
    assert transport.payloads == STOP_PAYLOADS


def test_malformed_sensor_frame_latches_and_stops(
    robot: Robot, transport: MockI2CTransport
) -> None:
    robot.arm()
    transport.line_sensor_length = 3
    with pytest.raises(SensorError):
        robot.read_line_sensors()
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.SENSOR_FAILURE
    assert transport.payloads == STOP_PAYLOADS


# -- 4. failed stops latch -----------------------------------------------------


def test_stop_failure_latches_and_disarms(
    robot: Robot, transport: MockI2CTransport
) -> None:
    robot.arm()
    transport.fail_all_writes = True
    with pytest.raises(I2CError):
        robot.stop()
    assert robot.armed is False
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.MOTOR_COMMUNICATION_FAILURE


def test_emergency_stop_persists_when_the_stop_fails(
    robot: Robot, transport: MockI2CTransport
) -> None:
    robot.arm()
    transport.fail_all_writes = True

    with pytest.raises(I2CError):
        robot.emergency_stop(FaultCode.OBSTACLE_DETECTED, "object 10 cm ahead")

    assert robot.emergency_stop_latched is True
    assert robot.armed is False
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.OBSTACLE_DETECTED
    assert "stop attempt failed" in robot.fault.message
    with pytest.raises(SafetyError):
        robot.forward(40)


def test_fault_latch_persists_when_the_stop_fails(
    robot: Robot, transport: MockI2CTransport
) -> None:
    robot.arm()
    transport.fail_all_writes = True

    record = robot.raise_fault(FaultCode.LINE_LOST, "no tape under any sensor")

    assert record.code is FaultCode.LINE_LOST
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.LINE_LOST
    assert "failed to stop the wheels" in robot.fault.message
    assert robot.armed is False


# -- 5. independent watchdog ---------------------------------------------------


def _watchdog_config(timeout_s: float = 0.05, period_s: float = 0.005) -> RobotConfig:
    return RobotConfig(
        safety=SafetyConfig(motion_timeout_s=timeout_s, watchdog_period_s=period_s)
    )


def test_watchdog_stops_without_tick(transport: MockI2CTransport, clock) -> None:
    robot = Robot(transport, _watchdog_config(), clock=clock)
    try:
        robot.arm()
        robot.forward(40)
        transport.reset()

        clock.advance(0.5)  # controller stalls: no further calls at all

        assert wait_until(
            lambda: robot.fault is not None
            and robot.fault.code is FaultCode.MOTION_TIMEOUT
        ), "watchdog never fired"
        assert transport.payloads == STOP_PAYLOADS
        assert robot.armed is False
        with pytest.raises(SafetyError):
            robot.forward(40)
    finally:
        robot.close()


def test_watchdog_does_not_fire_between_refreshes(
    transport: MockI2CTransport, clock
) -> None:
    robot = Robot(transport, _watchdog_config(timeout_s=0.2), clock=clock)
    try:
        robot.arm()
        for _ in range(5):
            clock.advance(0.05)
            robot.forward(40)
        time.sleep(0.05)
        assert robot.fault is None
    finally:
        robot.close()


def test_watchdog_stop_waits_for_the_in_flight_command(
    transport: MockI2CTransport, clock
) -> None:
    """The bus lock keeps a watchdog stop from splitting a four-wheel command."""
    entered = threading.Event()
    release = threading.Event()

    def hook(index: int) -> None:
        if index == 0 and not release.is_set():
            entered.set()
            release.wait(2.0)

    transport.before_write = hook
    robot = Robot(transport, _watchdog_config(0.05, 0.005), clock=clock)
    try:
        robot.arm()
        result: list[object] = []

        def drive() -> None:
            result.append(robot.forward(40))

        thread = threading.Thread(target=drive)
        thread.start()
        assert entered.wait(2.0), "drive never reached the bus"

        clock.advance(0.5)
        time.sleep(0.05)  # let the watchdog thread wake and block on the lock
        assert transport.writes == []

        release.set()
        thread.join(2.0)
        assert wait_until(lambda: len(transport.writes) >= 8)

        payloads = transport.payloads
        assert payloads[:4] == [(0, 0, 40), (1, 0, 40), (2, 0, 40), (3, 0, 40)]
        assert payloads[4:8] == STOP_PAYLOADS
        assert robot.fault is not None
        assert robot.fault.code is FaultCode.MOTION_TIMEOUT
    finally:
        release.set()
        robot.close()


def test_close_stops_the_wheels_and_the_watchdog_thread(
    transport: MockI2CTransport, clock
) -> None:
    robot = Robot(transport, _watchdog_config(), clock=clock)
    robot.arm()
    robot.forward(40)
    transport.reset()
    robot.close()
    assert transport.payloads == STOP_PAYLOADS
    assert robot.watchdog is not None
    assert robot.watchdog.running is False


# -- 6. external stop request --------------------------------------------------


def test_external_stop_request_refuses_motion_and_stops(
    transport: MockI2CTransport,
) -> None:
    requested = {"stop": False}
    robot = Robot(transport, stop_check=lambda: requested["stop"], watchdog=False)
    robot.arm()
    requested["stop"] = True

    with pytest.raises(SafetyError):
        robot.forward(40)

    assert transport.payloads == STOP_PAYLOADS
    assert robot.armed is False


def test_broken_stop_channel_is_treated_as_a_stop_request(
    transport: MockI2CTransport,
) -> None:
    def broken() -> bool:
        raise OSError("state directory is gone")

    robot = Robot(transport, stop_check=broken, watchdog=False)
    robot.arm()
    with pytest.raises(SafetyError):
        robot.forward(40)
    assert transport.payloads == STOP_PAYLOADS


# -- 7. strict wheel counts ----------------------------------------------------


@pytest.mark.parametrize(
    "bad",
    [
        (1.5, 0, 0, 0),
        (0, True, 0, 0),
        (0, 0, False, 0),
        ("40", 0, 0, 0),
    ],
)
def test_fractional_and_bool_wheel_counts_are_rejected(
    robot: Robot, transport: MockI2CTransport, bad
) -> None:
    robot.arm()
    with pytest.raises(SpeedLimitError):
        robot.drive_wheels(bad)
    assert transport.writes == []


def test_driver_rejects_fractional_counts(transport: MockI2CTransport) -> None:
    driver = MotorDriver(transport)
    for bad in ([1.0, 0, 0, 0], [0, True, 0, 0], [0, 0, 0, 2.5]):
        with pytest.raises(ValueError):
            driver.set_speeds(bad)
    assert transport.writes == []


# -- 8. bus mismatch -----------------------------------------------------------


def test_motor_and_sensor_bus_must_match() -> None:
    with pytest.raises(ConfigError):
        RobotConfig(
            motor=MotorConfig(bus=1), sensor=SensorConfig(bus=3)
        ).validate()
