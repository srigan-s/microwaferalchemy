"""Route execution: localization-driven arrival, dry runs, and failures."""

from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest

from waferbot import (
    ConfigError,
    FaultCode,
    MockI2CTransport,
    Robot,
    RobotConfig,
    SafetyError,
)
from waferbot.cli import MockEdgeProvider
from waferbot.errors import ExecutionError, LocalizationError, MapError
from waferbot.localization import Localization, ManualArrivalMonitor, MockArrivalMonitor
from waferbot.nav.executor import RouteExecutor
from waferbot.nav.graph import TrackMap
from waferbot.nav.planner import plan_route
from waferbot.navconfig import NavConfig
from waferbot.sensing import EdgeState

EXAMPLE = Path("maps/example_track.json")
STOP_PAYLOADS = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]


def load_track() -> TrackMap:
    return TrackMap.load(EXAMPLE)


def first_edge(track: TrackMap, route):
    for step in route.steps:
        if step.edge.edge_side is not None:
            try:
                return track.edge_state_for(
                    step.edge.edge_side, where=f"{step.source}->{step.destination}"
                )
            except MapError:
                return None
    return None


def build(route, track, *, clock, config=None):
    transport = MockI2CTransport()
    robot = Robot(transport, config or RobotConfig(), clock=clock, watchdog=False)
    transport.line_sensor_provider = MockEdgeProvider(
        transport, robot.config, first_edge(track, route) or EdgeState.BLACK_LEFT
    )
    return robot, transport


def make_executor(robot, track, *, clock, no_sleep, monitor, nav=None, **kwargs):
    return RouteExecutor(
        robot,
        track,
        nav=nav or NavConfig().with_follow(controller_enabled=True),
        arrival_monitor=monitor,
        clock=clock,
        sleep=no_sleep,
        **kwargs,
    )


# -- happy path --------------------------------------------------------------


def test_mock_route_a_plus_to_d_minus_via_b_minus(step_clock, no_sleep) -> None:
    track = load_track()
    route = plan_route(track, "A+", "D-", ["B-"])
    robot, transport = build(route, track, clock=step_clock)
    robot.arm()
    monitor = MockArrivalMonitor(polls_before_arrival=2, clock=step_clock)
    executor = make_executor(
        robot, track, clock=step_clock, no_sleep=no_sleep, monitor=monitor
    )

    result = executor.execute_route(
        route, enforce_map_readiness=False, allow_example_map=True
    )

    assert result.completed is True, result.reason
    assert result.route.route_id == route.route_id
    assert len(result.steps) == len(route.steps)
    assert all(step.completed for step in result.steps)
    assert any(step.action == "SWITCH_EDGE" for step in result.steps)
    assert result.final_localization is not None
    assert result.final_localization.node_id == "D-"
    assert robot.fault is None
    assert transport.motor_payloads[-4:] == STOP_PAYLOADS


def test_execute_route_requires_armed_robot(step_clock, no_sleep) -> None:
    track = load_track()
    route = plan_route(track, "A+", "B+")
    robot, _transport = build(route, track, clock=step_clock)
    executor = make_executor(
        robot,
        track,
        clock=step_clock,
        no_sleep=no_sleep,
        monitor=MockArrivalMonitor(polls_before_arrival=1, clock=step_clock),
    )
    with pytest.raises(SafetyError):
        executor.execute_route(route, enforce_map_readiness=False)


# -- dry run -----------------------------------------------------------------


def test_dry_run_touches_nothing(step_clock, no_sleep) -> None:
    track = load_track()
    route = plan_route(track, "A+", "D-", ["B-"])
    robot, transport = build(route, track, clock=step_clock)
    executor = make_executor(
        robot,
        track,
        clock=step_clock,
        no_sleep=no_sleep,
        monitor=MockArrivalMonitor(clock=step_clock),
    )
    result = executor.execute_route(route, dry_run=True)
    assert result.dry_run is True
    assert result.completed is True
    assert len(result.steps) == len(route.steps)
    assert transport.writes == []
    assert robot.armed is False


def test_dry_run_reports_uncalibrated_edge_sides(step_clock, no_sleep) -> None:
    data = json.loads(EXAMPLE.read_text(encoding="utf-8"))
    data["edge_side_map"] = {"positive": None, "negative": None}
    track = TrackMap.from_dict(data)
    route = plan_route(track, "A+", "D-", ["B-"])
    robot, _transport = build(route, track, clock=step_clock)
    executor = make_executor(
        robot,
        track,
        clock=step_clock,
        no_sleep=no_sleep,
        monitor=MockArrivalMonitor(clock=step_clock),
    )
    result = executor.execute_route(route, dry_run=True)
    assert result.completed is False
    assert "not calibrated" in result.reason


# -- map readiness -----------------------------------------------------------


def test_physical_execution_refuses_the_example_map(step_clock, no_sleep) -> None:
    track = load_track()
    route = plan_route(track, "A+", "B+")
    robot, transport = build(route, track, clock=step_clock)
    robot.arm()
    executor = make_executor(
        robot,
        track,
        clock=step_clock,
        no_sleep=no_sleep,
        monitor=MockArrivalMonitor(polls_before_arrival=1, clock=step_clock),
    )
    with pytest.raises(MapError):
        executor.execute_route(route, enforce_map_readiness=True)
    assert transport.writes == []


def test_uncalibrated_edge_map_is_refused_at_plan_time(step_clock, no_sleep) -> None:
    data = json.loads(EXAMPLE.read_text(encoding="utf-8"))
    data["is_example"] = False
    data["physical_validated"] = True
    data["edge_side_map"] = {"positive": None, "negative": None}
    track = TrackMap.from_dict(data)
    route = plan_route(track, "A+", "B+")
    robot, _transport = build(route, track, clock=step_clock)
    robot.arm()
    executor = make_executor(
        robot,
        track,
        clock=step_clock,
        no_sleep=no_sleep,
        monitor=MockArrivalMonitor(polls_before_arrival=1, clock=step_clock),
    )
    with pytest.raises(MapError):
        executor.execute_route(route)


# -- localization failures ---------------------------------------------------


def test_arrival_is_not_inferred_from_time(step_clock, no_sleep) -> None:
    track = load_track()
    route = plan_route(track, "A+", "B+")
    robot, _transport = build(route, track, clock=step_clock)
    robot.arm()

    class NeverArrives:
        def requires_stop(self) -> bool:
            return False

        def check(self, expected_node: str):
            return None

    executor = make_executor(
        robot, track, clock=step_clock, no_sleep=no_sleep, monitor=NeverArrives()
    )
    result = executor.execute_route(route, enforce_map_readiness=False)
    assert result.completed is False
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.LOCALIZATION_FAILURE


def test_start_node_must_be_confirmed(step_clock, no_sleep) -> None:
    track = load_track()
    route = plan_route(track, "A+", "B+")
    robot, transport = build(route, track, clock=step_clock)
    robot.arm()

    class WrongNode:
        def requires_stop(self) -> bool:
            return False

        def check(self, expected_node: str):
            return Localization(
                node_id="ZZ",
                marker_id="ZZ",
                confidence=1.0,
                timestamp=step_clock(),
                source="test",
            )

    executor = make_executor(
        robot, track, clock=step_clock, no_sleep=no_sleep, monitor=WrongNode()
    )
    result = executor.execute_route(route, enforce_map_readiness=False)
    assert result.completed is False
    assert result.steps == ()
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.LOCALIZATION_FAILURE
    # Only the shutdown stop blocks may reach the bus; no motion is allowed.
    assert transport.motor_payloads[-4:] == STOP_PAYLOADS
    assert all(payload[2] == 0 for payload in transport.motor_payloads)


def test_manual_confirmation_happens_with_motors_stopped(step_clock, no_sleep) -> None:
    track = load_track()
    route = plan_route(track, "A+", "B+")
    robot, transport = build(route, track, clock=step_clock)
    robot.arm()
    seen: list[bool] = []

    def prompt(_text: str) -> bool:
        seen.append(transport.motor_payloads[-4:] == STOP_PAYLOADS)
        return True

    monitor = ManualArrivalMonitor(prompt, clock=step_clock)
    executor = make_executor(
        robot, track, clock=step_clock, no_sleep=no_sleep, monitor=monitor
    )
    result = executor.execute_route(route, enforce_map_readiness=False)
    assert result.completed is True
    assert seen and all(seen), "operator prompts must happen with wheels stopped"


def test_no_localization_source_is_an_error(step_clock, no_sleep) -> None:
    track = load_track()
    route = plan_route(track, "A+", "B+")
    robot, _transport = build(route, track, clock=step_clock)
    robot.arm()
    executor = RouteExecutor(robot, track, clock=step_clock, sleep=no_sleep)
    with pytest.raises(LocalizationError):
        executor.execute_route(route, enforce_map_readiness=False)


# -- faults and bounded steps ------------------------------------------------


def test_sensor_fault_stops_the_route(step_clock, no_sleep) -> None:
    track = load_track()
    route = plan_route(track, "A+", "B+")
    robot, transport = build(route, track, clock=step_clock)
    robot.arm()
    transport.read_error = OSError("bus wedged")
    executor = make_executor(
        robot,
        track,
        clock=step_clock,
        no_sleep=no_sleep,
        monitor=MockArrivalMonitor(polls_before_arrival=2, clock=step_clock),
    )
    result = executor.execute_route(route, enforce_map_readiness=False)
    assert result.completed is False
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.SENSOR_FAILURE
    assert transport.motor_payloads[-4:] == STOP_PAYLOADS


def test_turn_step_requires_confirmation(step_clock, no_sleep) -> None:
    data = json.loads(EXAMPLE.read_text(encoding="utf-8"))
    # Make the dock reachable from the start and confirm the turn is bounded.
    track = TrackMap.from_dict(data)
    route = plan_route(track, "PARK", "D2")
    assert route.actions == (route.actions[0],) and route.actions[0].value == "TURN"
    robot, _transport = build(route, track, clock=step_clock)
    robot.arm()

    class NeverArrives:
        def requires_stop(self) -> bool:
            return False

        def check(self, expected_node: str):
            if expected_node == "PARK":
                return Localization(
                    node_id="PARK",
                    marker_id="PARK",
                    confidence=1.0,
                    timestamp=step_clock(),
                    source="test",
                )
            return None

    executor = make_executor(
        robot, track, clock=step_clock, no_sleep=no_sleep, monitor=NeverArrives()
    )
    result = executor.execute_route(route, enforce_map_readiness=False)
    assert result.completed is False
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.LOCALIZATION_FAILURE
    assert "elapsed time is not arrival evidence" in result.reason


def ready_track() -> TrackMap:
    data = json.loads(EXAMPLE.read_text(encoding="utf-8"))
    data["is_example"] = False
    data["physical_validated"] = True
    return TrackMap.from_dict(data)


def test_physical_execution_requires_a_measured_speed_factor(
    step_clock, no_sleep
) -> None:
    track = ready_track()
    route = plan_route(track, "A+", "B+")
    robot, transport = build(route, track, clock=step_clock)
    robot.arm()
    executor = make_executor(
        robot,
        track,
        clock=step_clock,
        no_sleep=no_sleep,
        monitor=MockArrivalMonitor(polls_before_arrival=2, clock=step_clock),
    )
    with pytest.raises(ConfigError) as excinfo:
        executor.execute_route(route, enforce_map_readiness=True)
    assert "counts_to_mps" in str(excinfo.value)
    assert transport.writes == []


def test_mock_execution_does_not_need_the_speed_factor(step_clock, no_sleep) -> None:
    track = load_track()
    route = plan_route(track, "A+", "B+")
    robot, _transport = build(route, track, clock=step_clock)
    robot.arm()
    executor = make_executor(
        robot,
        track,
        clock=step_clock,
        no_sleep=no_sleep,
        monitor=MockArrivalMonitor(polls_before_arrival=2, clock=step_clock),
    )
    result = executor.execute_route(
        route, enforce_map_readiness=False, allow_example_map=True
    )
    assert result.completed is True


def test_graph_speed_limit_caps_the_pwm_counts(step_clock, no_sleep) -> None:
    track = ready_track()
    route = plan_route(track, "A+", "B+")
    robot, transport = build(route, track, clock=step_clock)
    robot.arm()
    # 0.15 m/s with 0.01 m/s per count -> 15 counts maximum.
    nav = NavConfig(counts_to_mps=0.01, counts_to_mps_note="test").with_follow(
        controller_enabled=True
    )
    executor = make_executor(
        robot,
        track,
        clock=step_clock,
        no_sleep=no_sleep,
        monitor=MockArrivalMonitor(polls_before_arrival=2, clock=step_clock),
        nav=nav,
    )
    result = executor.execute_route(
        route, enforce_map_readiness=True, allow_example_map=True
    )
    assert result.completed is True
    driven = [payload[2] for payload in transport.motor_payloads if payload[2] > 0]
    assert driven, "no motor command was issued"
    assert max(driven) <= 15


def test_route_exceeding_duration_is_aborted(step_clock, no_sleep) -> None:
    track = load_track()
    route = plan_route(track, "A+", "D-", ["B-"])
    robot, _transport = build(route, track, clock=step_clock)
    robot.arm()
    nav = NavConfig(
        execution=type(NavConfig().execution)(max_route_duration_s=0.2)
    )
    executor = make_executor(
        robot,
        track,
        clock=step_clock,
        no_sleep=no_sleep,
        monitor=MockArrivalMonitor(polls_before_arrival=2, clock=step_clock),
        nav=nav,
    )
    result = executor.execute_route(
        route, enforce_map_readiness=False, allow_example_map=True
    )
    assert result.completed is False
    assert result.elapsed_s >= 0.0
