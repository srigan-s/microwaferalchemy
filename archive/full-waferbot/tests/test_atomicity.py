"""Stop/motion atomicity, watchdog lifecycle, and process-guard regressions.

These encode the review reproductions: a stop that latches while a command is in
flight must win, ``tick`` must not cancel a live deadline, and a cross-process
bus-lock timeout must latch a communication fault.
"""

from __future__ import annotations

import threading
import time

import pytest

from waferbot import (
    FaultCode,
    I2CError,
    MockI2CTransport,
    Robot,
    RobotConfig,
    SafetyConfig,
    SafetyError,
)
from waferbot.process import ProcessGuard

STOP_PAYLOADS = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]


def wait_until(predicate, timeout_s: float = 2.0, interval_s: float = 0.005) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval_s)
    return predicate()


def test_emergency_stop_latched_mid_command_wins(clock) -> None:
    """The review reproduction: e-stop fires while a four-wheel write is in flight."""
    transport = MockI2CTransport()
    robot = Robot(transport, RobotConfig(), clock=clock, watchdog=False)
    robot.arm()
    fired = threading.Event()
    stop_thread_done = threading.Event()

    def hook(index: int) -> None:
        if index == 1 and not fired.is_set():
            fired.set()

    transport.before_write = hook

    def stopper() -> None:
        fired.wait(2.0)
        try:
            robot.emergency_stop(FaultCode.OBSTACLE_DETECTED, "obstacle")
        finally:
            stop_thread_done.set()

    thread = threading.Thread(target=stopper)
    thread.start()
    robot.forward(60)
    assert stop_thread_done.wait(2.0)
    thread.join(2.0)

    assert robot.emergency_stop_latched is True
    assert robot.armed is False
    # The command's four writes may complete, but the stop must be last.
    assert transport.payloads[-4:] == STOP_PAYLOADS
    # And no new motion is possible while the emergency stop is latched.
    with pytest.raises(SafetyError):
        robot.forward(60)
    assert transport.payloads[-4:] == STOP_PAYLOADS


def test_external_stop_request_wins_under_the_transaction(clock) -> None:
    requested = {"stop": False}
    transport = MockI2CTransport()
    robot = Robot(
        transport, RobotConfig(), clock=clock, watchdog=False,
        stop_check=lambda: requested["stop"],
    )
    robot.arm()
    fired = threading.Event()

    def hook(index: int) -> None:
        if index == 1 and not fired.is_set():
            fired.set()
            requested["stop"] = True
            time.sleep(0.02)

    transport.before_write = hook
    robot.forward(60)
    assert fired.is_set()

    with pytest.raises(SafetyError):
        robot.forward(60)
    assert transport.payloads[-4:] == STOP_PAYLOADS
    assert all(
        payload[2] == 0
        for payload in transport.payloads[len(transport.payloads) - 4 :]
    )


def test_tick_does_not_cancel_a_live_deadline(robot, transport, clock) -> None:
    robot.arm()
    robot.forward(40)
    assert robot.watchdog is not None
    assert robot.watchdog.deadline is not None
    assert robot.safety.deadline is not None

    assert robot.tick() is False

    # The review reproduction: an ordinary early tick used to clear the deadline.
    assert robot.watchdog.deadline is not None
    assert robot.safety.deadline is not None
    assert robot.fault is None


def test_tick_expiry_latches_motion_timeout(robot, transport, clock) -> None:
    robot.arm()
    robot.forward(40)
    transport.reset()
    clock.advance(robot.safety.motion_timeout_s + 0.001)

    assert robot.tick() is True
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.MOTION_TIMEOUT
    assert transport.payloads[-4:] == STOP_PAYLOADS


def test_watchdog_with_real_thread_stops_and_does_not_cancel_new_deadlines() -> None:
    """Enabled-watchdog test with real timing (no watchdog=False shortcuts)."""
    transport = MockI2CTransport()
    config = RobotConfig(
        safety=SafetyConfig(motion_timeout_s=0.08, watchdog_period_s=0.005)
    )
    robot = Robot(transport, config)
    try:
        robot.arm()
        robot.forward(30)
        transport.reset()
        assert wait_until(
            lambda: robot.fault is not None
            and robot.fault.code is FaultCode.MOTION_TIMEOUT
        ), "watchdog never fired"
        assert transport.payloads[-4:] == STOP_PAYLOADS
    finally:
        robot.close()

    # A fresh robot can drive again: expiry did not poison the deadline machinery.
    transport2 = MockI2CTransport()
    robot2 = Robot(transport2, config)
    try:
        robot2.arm()
        robot2.forward(30)
        assert robot2.fault is None
        assert robot2.watchdog is not None
        assert robot2.watchdog.deadline is not None
    finally:
        robot2.close()


def test_run_command_refreshes_the_watchdog_over_a_long_hold() -> None:
    transport = MockI2CTransport()
    config = RobotConfig(
        safety=SafetyConfig(motion_timeout_s=0.15, watchdog_period_s=0.005)
    )
    robot = Robot(transport, config)
    try:
        robot.arm()
        elapsed = robot.run_command(
            lambda: robot.forward(25), 0.6, refresh_period_s=0.05
        )
        assert elapsed >= 0.5
        assert robot.fault is None, "watchdog fired during a refreshing hold"
        assert transport.payloads[-4:] == STOP_PAYLOADS
    finally:
        robot.close()


def test_run_command_rejects_invalid_durations(robot) -> None:
    robot.arm()
    for bad in (float("nan"), float("inf"), -1.0, True, "1"):
        with pytest.raises(ValueError):
            robot.run_command(lambda: robot.forward(10), bad)


def test_bus_lock_timeout_latches_communication_fault(clock) -> None:
    transport = MockI2CTransport()

    def timing_out_guard():
        raise TimeoutError("bus busy")

    robot = Robot(
        transport,
        RobotConfig(),
        clock=clock,
        watchdog=False,
        bus_guard=timing_out_guard,
    )
    robot.arm()
    with pytest.raises(I2CError):
        robot.forward(40)
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.MOTOR_COMMUNICATION_FAILURE
    assert robot.armed is False
    assert transport.writes == []


def test_nested_bus_guard_is_reentrant_within_a_process(tmp_path) -> None:
    guard = ProcessGuard(tmp_path)
    started = time.monotonic()
    with guard.bus_guard(timeout_s=0.2):
        assert guard.holds_bus_lock is True
        with guard.bus_guard(timeout_s=0.2):
            assert guard.holds_bus_lock is True
    assert guard.holds_bus_lock is False
    assert time.monotonic() - started < 0.2, "nested acquisition deadlocked"


def test_transport_is_closed_on_lifecycle_cleanup(clock) -> None:
    transport = MockI2CTransport()
    robot = Robot(transport, RobotConfig(), clock=clock, watchdog=False)
    robot.arm()
    robot.close()
    assert transport.closed is True


def test_started_watchdog_is_stopped_by_close() -> None:
    transport = MockI2CTransport()
    robot = Robot(transport, RobotConfig())
    robot.arm()
    assert robot.watchdog is not None and robot.watchdog.running is True
    robot.close()
    assert robot.watchdog.running is False

