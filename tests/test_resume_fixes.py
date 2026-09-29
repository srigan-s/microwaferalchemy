"""Regression tests for the September 28 resume contract.

Covers the concrete defects listed in RESUME.md: refreshed calibration
movements, stop-failure propagation, atomic watchdog expiry, switch motion
budgeting (manual pauses are free), TURN heading conventions, and marker-id
mapping reaching the localization policies.
"""

from __future__ import annotations

import json
import threading
import time

import pytest

from waferbot import (
    ConfigError,
    FaultCode,
    I2CError,
    MockI2CTransport,
    Robot,
    RobotConfig,
    SafetyConfig,
)
from waferbot.calibration import WheelCalibrator
from waferbot.localization import Localization
from waferbot.nav.executor import RouteExecutor
from waferbot.nav.graph import TrackMap, TravelDirection
from waferbot.nav.planner import plan_route
from waferbot.navconfig import FollowConfig, NavConfig, SwitchConfig
from waferbot.sensing import EdgeState, SwitchAuthorization, edge_byte_for
from waferbot.sensing.switching import EdgeSwitcher
from waferbot.simulation import Pose, TapeModelConfig, run_closed_loop
from waferbot.watchdog import MotionWatchdog

LEFT = edge_byte_for(EdgeState.BLACK_LEFT)
RIGHT = edge_byte_for(EdgeState.BLACK_RIGHT)
STOP_PAYLOADS = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]


def make_robot(sequence=None, *, clock=None, watchdog=False, sleep=None, config=None):
    transport = MockI2CTransport(line_sensor_sequence=list(sequence or []))
    kwargs = {"watchdog": watchdog}
    if clock is not None:
        kwargs["clock"] = clock
    if sleep is not None:
        kwargs["sleep"] = sleep
    robot = Robot(transport, config or RobotConfig(), **kwargs)
    return robot, transport


# -- 2. refreshed calibration movements and stop-failure propagation ----------


def test_run_command_rejects_a_period_longer_than_the_watchdog(clock, no_sleep):
    robot, _transport = make_robot(
        [LEFT] * 10,
        clock=clock,
        config=RobotConfig(safety=SafetyConfig(motion_timeout_s=0.1, watchdog_period_s=0.01)),
    )
    robot.arm()
    with pytest.raises(ValueError) as excinfo:
        robot.run_command(
            lambda: robot.forward(20), 0.5, refresh_period_s=0.2
        )
    assert "shorter than the motion watchdog" in str(excinfo.value)


def test_run_command_reports_a_failed_final_stop(clock, no_sleep):
    robot, transport = make_robot([LEFT] * 10, clock=clock, sleep=no_sleep)
    robot.arm()
    transport.fail_stops_after = 0
    with pytest.raises(I2CError):
        robot.run_command(lambda: robot.forward(20), 0.2, refresh_period_s=0.05)
    assert robot.fault is not None


def test_run_command_preserves_a_primary_error_when_the_stop_also_fails(clock, no_sleep):
    robot, transport = make_robot([LEFT] * 10, clock=clock, sleep=no_sleep)
    robot.arm()
    transport.fail_stops_after = 0

    def exploding_command():
        raise RuntimeError("primary command failure")

    with pytest.raises(RuntimeError) as excinfo:
        robot.run_command(exploding_command, 0.2, refresh_period_s=0.05)
    assert "primary command failure" in str(excinfo.value)
    assert robot.fault is not None
    assert "final stop failed" in robot.fault.message


def test_direction_checks_refresh_under_an_enabled_watchdog(no_sleep):
    transport = MockI2CTransport()
    config = RobotConfig(
        safety=SafetyConfig(motion_timeout_s=0.1, watchdog_period_s=0.005)
    )
    robot = Robot(transport, config)
    try:
        robot.arm()
        calibrator = WheelCalibrator(robot, ask=lambda _p: "yes", sleep=no_sleep)
        checks = calibrator.check_directions(speed=25, duration_s=0.3)
        assert robot.fault is None, "watchdog fired during a refreshed check"
        assert set(checks) == {
            "forward",
            "backward",
            "strafe_left",
            "strafe_right",
            "rotate_left",
            "rotate_right",
        }
        assert transport.payloads[-4:] == STOP_PAYLOADS
    finally:
        robot.close()


# -- 3. atomic watchdog expiry ------------------------------------------------


def test_consume_expired_is_atomic(clock):
    watchdog = MotionWatchdog(lambda: None, clock=clock, poll_interval_s=0.01)
    assert watchdog.consume_expired() is False       # never armed
    watchdog.refresh(0.5)
    assert watchdog.consume_expired() is False       # live
    clock.advance(0.6)
    assert watchdog.consume_expired() is True        # claims the expiry once
    assert watchdog.consume_expired() is False       # and only once


def test_refresh_after_a_claimed_expiry_prevents_a_stale_timeout(clock, no_sleep):
    robot, transport = make_robot(
        [LEFT] * 10,
        clock=clock,
        config=RobotConfig(safety=SafetyConfig(motion_timeout_s=0.1, watchdog_period_s=0.01)),
    )
    robot.arm()
    robot.forward(40)
    transport.reset()
    clock.advance(0.2)          # the deadline has passed
    robot.forward(40)           # ... but the controller refreshes in time
    robot._on_motion_timeout("test")
    assert robot.fault is None
    # No stop blocks were written by the stale timeout handler: the last command
    # is still the refreshed forward command.
    assert transport.payloads[-4:] == [(0, 0, 40), (1, 0, 40), (2, 0, 40), (3, 0, 40)]


def test_continuous_refresh_never_trips_the_real_watchdog():
    transport = MockI2CTransport()
    config = RobotConfig(
        safety=SafetyConfig(motion_timeout_s=0.15, watchdog_period_s=0.005)
    )
    robot = Robot(transport, config)
    try:
        robot.arm()
        deadline = time.monotonic() + 0.6
        while time.monotonic() < deadline:
            robot.forward(30)
            time.sleep(0.02)
        assert robot.fault is None, "watchdog produced a spurious timeout"
    finally:
        robot.close()


def test_watchdog_fires_once_refresh_stops():
    transport = MockI2CTransport()
    config = RobotConfig(
        safety=SafetyConfig(motion_timeout_s=0.08, watchdog_period_s=0.005)
    )
    robot = Robot(transport, config)
    try:
        robot.arm()
        robot.forward(30)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and robot.fault is None:
            time.sleep(0.01)
        assert robot.fault is not None
        assert robot.fault.code is FaultCode.MOTION_TIMEOUT
        assert transport.payloads[-4:] == STOP_PAYLOADS
    finally:
        robot.close()


# -- 4. switch motion budget --------------------------------------------------


def test_manual_confirmation_time_does_not_consume_the_motion_budget(step_clock, no_sleep):
    # Model fast clock reads, below the 250ms sensor sampling-gap limit.
    step_clock.step = 0.005
    transport = MockI2CTransport(line_sensor_sequence=[LEFT] * 10 + [RIGHT] * 80)
    robot = Robot(transport, RobotConfig(), clock=step_clock, watchdog=False)
    robot.arm()
    prompts: list[str] = []

    def confirm(prompt: str) -> bool:
        prompts.append(prompt)
        # A pause longer than the whole motion budget: it must not count against
        # the manoeuvre.
        step_clock.advance(30.0)
        return True

    switcher = EdgeSwitcher(
        robot,
        config=SwitchConfig(max_travel_m=1.5),
        clock=step_clock,
        sleep=no_sleep,
        confirm=confirm,
        counts_to_mps=0.01,
        require_distance_bound=True,
    )
    authorization = SwitchAuthorization(
        authorized=True,
        route_id="r",
        source_node="B2",
        source_edge=EdgeState.BLACK_LEFT,
        target_edge=EdgeState.BLACK_RIGHT,
        location_id="B2",
        destination_location_id="B3",
    )
    result = switcher.switch_edge(
        EdgeState.BLACK_LEFT,
        EdgeState.BLACK_RIGHT,
        authorization=authorization,
        location_id="B2",
    )
    assert result.completed is True, result.reason
    assert prompts, "the operator must be asked before crossing"


def test_switch_revalidates_the_edge_after_a_manual_pause(step_clock, no_sleep):
    # The tape disappears while the operator is being asked to confirm.
    transport = MockI2CTransport(line_sensor_sequence=[LEFT] * 4 + [0xFF] * 200)
    robot = Robot(transport, RobotConfig(), clock=step_clock, watchdog=False)
    robot.arm()
    switcher = EdgeSwitcher(
        robot,
        config=SwitchConfig(),
        clock=step_clock,
        sleep=no_sleep,
        confirm=lambda _prompt: True,
    )
    authorization = SwitchAuthorization(
        authorized=True,
        route_id="r",
        source_node="B2",
        source_edge=EdgeState.BLACK_LEFT,
        target_edge=EdgeState.BLACK_RIGHT,
        location_id="B2",
    )
    result = switcher.switch_edge(
        EdgeState.BLACK_LEFT,
        EdgeState.BLACK_RIGHT,
        authorization=authorization,
        location_id="B2",
    )
    assert result.completed is False
    assert "stopped for confirmation" in result.reason
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.LOCALIZATION_FAILURE


def test_route_deadline_reaches_the_switcher(step_clock, no_sleep):
    transport = MockI2CTransport(line_sensor_sequence=[LEFT] * 20 + [RIGHT] * 60)
    robot = Robot(transport, RobotConfig(), clock=step_clock, watchdog=False)
    robot.arm()
    switcher = EdgeSwitcher(
        robot,
        config=SwitchConfig(),
        clock=step_clock,
        sleep=no_sleep,
        confirm=lambda _prompt: True,
    )
    authorization = SwitchAuthorization(
        authorized=True,
        route_id="r",
        source_node="B2",
        source_edge=EdgeState.BLACK_LEFT,
        target_edge=EdgeState.BLACK_RIGHT,
        location_id="B2",
    )
    expired = step_clock() - 1.0
    result = switcher.switch_edge(
        EdgeState.BLACK_LEFT,
        EdgeState.BLACK_RIGHT,
        authorization=authorization,
        location_id="B2",
        deadline=expired,
    )
    assert result.completed is False
    # No meaningful manoeuvre time (only the two immediately-expired phase
    # checks) and no wheel was ever commanded to move.
    assert result.travelled_s <= 0.5
    assert all(payload[2] == 0 for payload in transport.motor_payloads)


# -- 5. TURN heading convention and marker mapping ----------------------------


def small_turn_map(*, direction: str | None = None) -> dict:
    node = lambda node_id, heading, marker: {  # noqa: E731
        "node_id": node_id,
        "station_id": node_id,
        "x": 0.0,
        "y": 0.0,
        "heading": heading,
        "edge_side": None,
        "direction": None,
        "node_type": "station",
        "marker_id": marker,
    }
    edge = {
        "source": "S",
        "destination": "T",
        "distance_m": 0.05,
        "speed_limit_mps": 0.05,
        "action": "TURN",
        "edge_side": None,
        "turn_time_s": 0.4,
        "enabled": True,
    }
    if direction is not None:
        edge["direction"] = direction
    return {
        "schema_version": 1,
        "name": "turn-bench",
        "is_example": True,
        "physical_validated": False,
        "edge_side_map": {"positive": "BLACK_LEFT", "negative": "BLACK_RIGHT"},
        "nodes": [node("S", 0.0, "QR-S"), node("T", 90.0, "QR-T")],
        "edges": [edge],
    }


@pytest.mark.parametrize(
    ("start_heading", "end_heading", "expect_left"),
    [
        (0.0, 90.0, True),
        (0.0, 270.0, False),
        (350.0, 10.0, True),    # wraparound, short way round
        (10.0, 350.0, False),
        (0.0, 180.0, False),    # exact 180 resolves clockwise by convention
    ],
)
def test_turn_uses_the_signed_map_heading_delta(
    step_clock, no_sleep, start_heading, end_heading, expect_left
):
    data = small_turn_map()
    data["nodes"][0]["heading"] = start_heading
    data["nodes"][1]["heading"] = end_heading
    track = TrackMap.from_dict(data)
    route = plan_route(track, "S", "T")
    transport = MockI2CTransport()
    robot = Robot(transport, RobotConfig(), clock=step_clock, watchdog=False)
    robot.arm()
    executor = RouteExecutor(
        robot,
        track,
        nav=NavConfig(),
        arrival_monitor=None,
        confirm=lambda _prompt: True,
        clock=step_clock,
        sleep=no_sleep,
    )
    result = executor.execute_route(route, enforce_map_readiness=False)
    assert result.completed is True, result.reason
    groups = [
        transport.motor_payloads[i : i + 4]
        for i in range(0, len(transport.motor_payloads), 4)
    ]
    first_command = next(group for group in groups if any(p[2] > 0 for p in group))
    left_signs = [1 if payload[1] == 0 else -1 for payload in first_command]
    if expect_left:
        # rotate_left: left wheels backwards, right wheels forwards.
        assert left_signs == [-1, -1, 1, 1]
    else:
        assert left_signs == [1, 1, -1, -1]


def test_turn_with_reverse_direction_is_rejected_by_map_validation():
    data = small_turn_map(direction="reverse")
    with pytest.raises(Exception) as excinfo:
        TrackMap.from_dict(data)
    assert "TURN" in str(excinfo.value)


def test_marker_id_mapping_confirms_arrival(step_clock, no_sleep):
    """A localizer reporting only a marker id must still confirm the node."""
    track = TrackMap.from_dict(small_turn_map())
    assert track.marker_to_node() == {"QR-S": "S", "QR-T": "T"}
    route = plan_route(track, "S", "T")
    transport = MockI2CTransport()
    robot = Robot(transport, RobotConfig(), clock=step_clock, watchdog=False)
    robot.arm()

    class MarkerOnlyMonitor:
        def requires_stop(self) -> bool:
            return False

        def check(self, expected_node: str):
            marker = {"S": "QR-S", "T": "QR-T"}.get(expected_node)
            if marker is None:
                return None
            return Localization(
                node_id=None,
                marker_id=marker,
                confidence=1.0,
                timestamp=step_clock(),
                source="marker",
            )

    executor = RouteExecutor(
        robot,
        track,
        nav=NavConfig(),
        arrival_monitor=MarkerOnlyMonitor(),
        clock=step_clock,
        sleep=no_sleep,
    )
    result = executor.execute_route(route, enforce_map_readiness=False)
    assert result.completed is True, result.reason
    assert result.final_localization is not None
    assert result.final_localization.marker_id == "QR-T"


def test_switch_total_moving_time_honours_max_travel(clock):
    # A clock advanced only by sleeps measures actual commanded moving time,
    # independently of how often implementation code reads the clock.
    transport = MockI2CTransport(line_sensor_sequence=[LEFT] * 6 + [0x0F] * 100)
    robot = Robot(transport, clock=clock, sleep=clock.advance, watchdog=False)
    robot.arm()
    switcher = EdgeSwitcher(
        robot, config=SwitchConfig(max_travel_s=0.12), clock=clock,
        sleep=clock.advance, confirm=lambda _: True,
    )
    result = switcher.switch_edge(
        EdgeState.BLACK_LEFT, EdgeState.BLACK_RIGHT,
        authorization=SwitchAuthorization(authorized=True, route_id="bounded",
            source_node="B2", source_edge=EdgeState.BLACK_LEFT,
            target_edge=EdgeState.BLACK_RIGHT, location_id="B2"), location_id="B2",
    )
    assert not result.completed
    assert result.travelled_s <= 0.12 + 1e-9
    assert any(payload[2] for payload in transport.payloads)
    assert transport.payloads[-4:] == STOP_PAYLOADS


def test_switch_never_resumes_if_operator_moves_off_target(clock):
    transport = MockI2CTransport(line_sensor_sequence=[LEFT] * 6 + [RIGHT] * 100)
    robot = Robot(transport, clock=clock, sleep=clock.advance, watchdog=False)
    robot.arm()
    pause_write_index = []
    def confirm(prompt):
        if "after crossing" in prompt:
            pause_write_index.append(len(transport.payloads))
            transport.line_sensor_sequence.clear()
            transport.line_sensor_byte = LEFT
            clock.advance(20.0)
        return True
    switcher = EdgeSwitcher(robot, clock=clock, sleep=clock.advance, confirm=confirm)
    result = switcher.switch_edge(
        EdgeState.BLACK_LEFT, EdgeState.BLACK_RIGHT,
        authorization=SwitchAuthorization(authorized=True, route_id="recheck",
            source_node="B2", source_edge=EdgeState.BLACK_LEFT,
            target_edge=EdgeState.BLACK_RIGHT, location_id="B2",
            destination_location_id="B3"), location_id="B2",
    )
    assert pause_write_index
    assert not result.completed
    assert "after destination confirmation" in result.reason
    assert all(p[2] == 0 for p in transport.payloads[pause_write_index[0]:])
