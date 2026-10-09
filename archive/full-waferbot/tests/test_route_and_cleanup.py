"""Route/map validation, per-edge switch geometry, and honest cleanup results."""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path

import pytest

from waferbot import ConfigError, FaultCode, MockI2CTransport, Robot, RobotConfig
from waferbot.cli import MockEdgeProvider
from waferbot.errors import ExecutionError, MapError
from waferbot.localization import Localization, MockArrivalMonitor
from waferbot.nav.executor import RouteExecutor
from waferbot.nav.graph import MapEdge, TrackMap
from waferbot.nav.planner import Planner, Route, RouteStep, plan_route
from waferbot.navconfig import FollowConfig, GeometryConfig, NavConfig, SwitchConfig
from waferbot.sensing import EdgeState, edge_byte_for
from waferbot.sensing.follower import EdgeFollower, FollowStopReason
from waferbot.sensing.switching import EdgeSwitcher, SwitchAuthorization

EXAMPLE = Path("maps/example_track.json")
LEFT = edge_byte_for(EdgeState.BLACK_LEFT)
RIGHT = edge_byte_for(EdgeState.BLACK_RIGHT)
STOP_PAYLOADS = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]


def example_dict() -> dict:
    return json.loads(EXAMPLE.read_text(encoding="utf-8"))


def load_track() -> TrackMap:
    return TrackMap.load(EXAMPLE)


def first_edge(track: TrackMap, route) -> EdgeState | None:
    for step in route.steps:
        if step.edge.edge_side is not None:
            try:
                return track.edge_state_for(
                    step.edge.edge_side, where=f"{step.source}->{step.destination}"
                )
            except MapError:
                return None
    return None


def build(route, track, *, clock, config=None, transport=None):
    transport = transport or MockI2CTransport()
    robot = Robot(transport, config or RobotConfig(), clock=clock, watchdog=False)
    transport.line_sensor_provider = MockEdgeProvider(
        transport, robot.config, first_edge(track, route) or EdgeState.BLACK_LEFT
    )
    return robot, transport


def make_executor(robot, track, *, clock, no_sleep, monitor, nav=None, **kwargs):
    return RouteExecutor(
        robot,
        track,
        nav=nav or NavConfig(),
        arrival_monitor=monitor,
        clock=clock,
        sleep=no_sleep,
        **kwargs,
    )


# -- route validation --------------------------------------------------------


def test_forged_route_edge_is_refused_before_motion(step_clock, no_sleep) -> None:
    track = load_track()
    route = plan_route(track, "A+", "B+")
    forged_edge = MapEdge(
        source="A+",
        destination="C-",
        distance_m=1.0,
        speed_limit_mps=0.15,
        action=route.steps[0].edge.action,
        edge_side=route.steps[0].edge.edge_side,
    )
    forged = replace(
        route,
        destination_node="C-",
        nodes=("A+", "C-"),
        steps=(replace(route.steps[0], edge=forged_edge),),
    )
    robot, transport = build(route, track, clock=step_clock)
    robot.arm()
    executor = make_executor(
        robot, track, clock=step_clock, no_sleep=no_sleep,
        monitor=MockArrivalMonitor(polls_before_arrival=1, clock=step_clock),
    )
    with pytest.raises(ExecutionError) as excinfo:
        executor.execute_route(forged, enforce_map_readiness=False)
    assert "current map" in str(excinfo.value)
    assert transport.writes == []


def test_stale_route_values_are_refused(step_clock, no_sleep) -> None:
    track = load_track()
    route = plan_route(track, "A+", "B+")
    stale_edge = replace(route.steps[0].edge, distance_m=99.0)
    stale = replace(
        route,
        steps=(replace(route.steps[0], edge=stale_edge),),
    )
    robot, transport = build(route, track, clock=step_clock)
    robot.arm()
    executor = make_executor(
        robot, track, clock=step_clock, no_sleep=no_sleep,
        monitor=MockArrivalMonitor(polls_before_arrival=1, clock=step_clock),
    )
    with pytest.raises(ExecutionError) as excinfo:
        executor.execute_route(stale, enforce_map_readiness=False)
    assert "stale or forged" in str(excinfo.value)
    assert transport.writes == []


def test_route_node_sequence_must_match_steps(step_clock, no_sleep) -> None:
    track = load_track()
    route = plan_route(track, "A+", "B+")
    broken = replace(route, nodes=("A+", "ZZ"))
    robot, _transport = build(route, track, clock=step_clock)
    robot.arm()
    executor = make_executor(
        robot, track, clock=step_clock, no_sleep=no_sleep,
        monitor=MockArrivalMonitor(polls_before_arrival=1, clock=step_clock),
    )
    with pytest.raises(ExecutionError):
        executor.execute_route(broken, enforce_map_readiness=False)


def test_required_waypoint_must_appear_in_order(step_clock, no_sleep) -> None:
    track = load_track()
    route = plan_route(track, "A+", "B-", ["B-"])
    tampered = replace(route, required_waypoints=("D-",))
    robot, _transport = build(route, track, clock=step_clock)
    robot.arm()
    executor = make_executor(
        robot, track, clock=step_clock, no_sleep=no_sleep,
        monitor=MockArrivalMonitor(polls_before_arrival=1, clock=step_clock),
    )
    with pytest.raises(ExecutionError) as excinfo:
        executor.execute_route(tampered, enforce_map_readiness=False)
    assert "waypoints" in str(excinfo.value)


def test_disabled_connection_is_refused(step_clock, no_sleep) -> None:
    data = example_dict()
    data["edges"][0]["enabled"] = False
    track = TrackMap.from_dict(data)
    full = load_track()
    route = plan_route(full, "A+", "B+")
    robot, transport = build(route, full, clock=step_clock)
    robot.arm()
    executor = make_executor(
        robot, track, clock=step_clock, no_sleep=no_sleep,
        monitor=MockArrivalMonitor(polls_before_arrival=1, clock=step_clock),
    )
    with pytest.raises(ExecutionError) as excinfo:
        executor.execute_route(route, enforce_map_readiness=False)
    assert "disabled" in str(excinfo.value)
    assert transport.writes == []


def test_reverse_follow_edge_is_refused(step_clock, no_sleep) -> None:
    data = example_dict()
    follow = next(e for e in data["edges"] if e["action"] == "FOLLOW_EDGE")
    follow["direction"] = "reverse"
    for node in data["nodes"]:
        if node["node_id"] == follow["destination"]:
            node["direction"] = "reverse"
    track = TrackMap.from_dict(data)
    route = plan_route(track, "A+", "B+")
    assert route.steps[0].edge.direction is not None
    robot, transport = build(route, track, clock=step_clock)
    robot.arm()
    executor = make_executor(
        robot, track, clock=step_clock, no_sleep=no_sleep,
        monitor=MockArrivalMonitor(polls_before_arrival=1, clock=step_clock),
    )
    with pytest.raises(ExecutionError) as excinfo:
        executor.execute_route(route, enforce_map_readiness=False)
    assert "reverse" in str(excinfo.value)
    assert transport.writes == []


def test_spatial_stop_edge_is_rejected_by_map_validation():
    data = example_dict()
    stop = next(e for e in data["edges"] if e["action"] == "STOP")
    stop["source"] = "D+"
    stop["destination"] = "PARK"
    with pytest.raises(MapError) as excinfo:
        TrackMap.from_dict(data)
    assert "teleport" in str(excinfo.value)


def test_self_stop_edge_is_valid_and_stationary():
    track = load_track()
    stop = next(edge for edge in track.edges if edge.action.value == "STOP")
    assert stop.source == stop.destination
    assert stop.distance_m == 0.0


def test_edge_side_must_match_its_nodes():
    data = example_dict()
    edge = next(e for e in data["edges"] if e["action"] == "FOLLOW_EDGE")
    edge["edge_side"] = "negative"  # A+/B+ are positive nodes
    with pytest.raises(MapError) as excinfo:
        TrackMap.from_dict(data)
    message = str(excinfo.value)
    assert "edge_side" in message or "changes edge side" in message


def test_direction_convention_is_enforced():
    data = example_dict()
    edge = next(e for e in data["edges"] if e["action"] == "FOLLOW_EDGE")
    edge["direction"] = "reverse"
    with pytest.raises(MapError) as excinfo:
        TrackMap.from_dict(data)
    assert "direction" in str(excinfo.value)


# -- planner correctness -----------------------------------------------------


def test_astar_priority_does_not_double_count_g():
    """A* must return the optimal cost, not a g-doubled ordering artefact."""
    data = {
        "schema_version": 1,
        "name": "cost-check",
        "is_example": False,
        "physical_validated": True,
        "edge_side_map": {"positive": "BLACK_LEFT", "negative": "BLACK_RIGHT"},
        "nodes": [
            {"node_id": "S", "station_id": "S", "x": 0.0, "y": 0.0, "heading": 0.0, "edge_side": "positive", "direction": "forward", "node_type": "station", "marker_id": "S"},
            {"node_id": "M", "station_id": "M", "x": 1.0, "y": 0.0, "heading": 0.0, "edge_side": "positive", "direction": "forward", "node_type": "station", "marker_id": "M"},
            {"node_id": "T", "station_id": "T", "x": 2.0, "y": 0.0, "heading": 0.0, "edge_side": "positive", "direction": "forward", "node_type": "station", "marker_id": "T"},
        ],
        "edges": [
            {"source": "S", "destination": "T", "distance_m": 5.0, "speed_limit_mps": 0.1, "action": "FOLLOW_EDGE", "edge_side": "positive", "enabled": True},
            {"source": "S", "destination": "M", "distance_m": 1.0, "speed_limit_mps": 0.1, "action": "FOLLOW_EDGE", "edge_side": "positive", "enabled": True},
            {"source": "M", "destination": "T", "distance_m": 1.0, "speed_limit_mps": 0.1, "action": "FOLLOW_EDGE", "edge_side": "positive", "enabled": True},
        ],
    }
    track = TrackMap.from_dict(data)
    dijkstra = Planner(track, algorithm="dijkstra").plan_route("S", "T")
    astar = Planner(track, algorithm="astar").plan_route("S", "T")
    assert dijkstra.total_cost_s == pytest.approx(20.0)  # 2 m at 0.1 m/s
    assert astar.total_cost_s == pytest.approx(dijkstra.total_cost_s)
    assert astar.nodes == ("S", "M", "T")


def test_stop_edges_participate_in_metric_consistency():
    data = example_dict()
    stop = next(e for e in data["edges"] if e["action"] == "STOP")
    # Self-loop with zero distance is consistent; give it a spatial jump by
    # moving the node coordinates apart and the map must be rejected outright.
    stop["source"] = "PARK"
    stop["destination"] = "D-"
    with pytest.raises(MapError):
        TrackMap.from_dict(data)


def test_adjacency_index_is_a_real_adjacency_list():
    track = load_track()
    index = track._adjacency_index()
    assert set(index) == {node.node_id for node in track.nodes}
    assert [edge.destination for edge in index["A+"]] == ["B+"]
    assert all(edge.source == "A+" for edge in index["A+"])


# -- per-edge switch geometry ------------------------------------------------


def test_diagonal_edge_requires_measured_geometry(step_clock, no_sleep) -> None:
    data = example_dict()
    switch = next(e for e in data["edges"] if e["action"] == "SWITCH_EDGE")
    switch["crossing_angle_deg"] = 18.0
    track = TrackMap.from_dict(data)
    route = plan_route(track, "B+", "B-")
    robot, transport = build(route, track, clock=step_clock)
    robot.arm()
    executor = make_executor(
        robot, track, clock=step_clock, no_sleep=no_sleep,
        monitor=MockArrivalMonitor(polls_before_arrival=2, clock=step_clock),
    )
    with pytest.raises(ExecutionError) as excinfo:
        executor.execute_route(route, enforce_map_readiness=False)
    assert "measured geometry" in str(excinfo.value)
    assert transport.writes == []


def test_diagonal_edge_runs_with_measured_geometry(step_clock, no_sleep) -> None:
    data = example_dict()
    switch = next(e for e in data["edges"] if e["action"] == "SWITCH_EDGE")
    switch["crossing_angle_deg"] = 18.0
    track = TrackMap.from_dict(data)
    route = plan_route(track, "B+", "B-")
    robot, transport = build(route, track, clock=step_clock)
    robot.arm()
    geometry = GeometryConfig(
        tape_width_m=0.02,
        sensor_spacing_m=0.012,
        sensor_detection_width_m=0.004,
        available_crossing_distance_m=0.2,
        safety_margin_m=0.01,
        robot_width_m=0.15,
        switching_speed_mps=0.1,
        sampling_rate_hz=20.0,
        lateral_clearance_m=0.3,
        required_confirmations=3,
        measured=True,
    )
    nav = NavConfig(
        geometry=geometry,
        switch=SwitchConfig(mode="diagonal", crossing_angle_deg=18.0),
    )
    executor = make_executor(
        robot, track, clock=step_clock, no_sleep=no_sleep,
        monitor=MockArrivalMonitor(polls_before_arrival=2, clock=step_clock),
        nav=nav,
    )
    result = executor.execute_route(
        route, enforce_map_readiness=False, allow_example_map=True
    )
    assert result.completed is True, result.reason
    switch_step = next(s for s in result.steps if s.action == "SWITCH_EDGE")
    assert switch_step.completed is True
    assert switch_step.detail["geometry"]["unambiguous_geometry"] is True


def test_diagonal_blend_never_exceeds_its_ceiling():
    wheels = EdgeSwitcher._blend(30, EdgeState.BLACK_LEFT, 20, 35)
    assert max(abs(value) for value in wheels) <= 35
    # The angle is preserved: the lateral/forward ratio survives the scaling.
    unscaled = (30 - 20, 30 + 20)
    scaled = (wheels[0], wheels[1])

    def ratio(pair):
        forward = (pair[0] + pair[1]) / 2
        lateral = (pair[1] - pair[0]) / 2
        return lateral / forward

    assert ratio(scaled) == pytest.approx(ratio(unscaled), abs=0.05)


def test_sub_one_count_ceiling_is_refused_not_rounded_up():
    with pytest.raises(ConfigError):
        EdgeSwitcher._blend(0, EdgeState.BLACK_LEFT, 20, 0)
    with pytest.raises(ConfigError):
        EdgeSwitcher._blend(1, EdgeState.BLACK_LEFT, 0, 0)


def test_physical_switch_requires_a_measured_travel_budget(step_clock, no_sleep) -> None:
    robot, transport = build(
        plan_route(load_track(), "A+", "B+"), load_track(), clock=step_clock
    )
    robot.arm()
    switcher = EdgeSwitcher(
        robot,
        config=SwitchConfig(),
        clock=step_clock,
        sleep=no_sleep,
        location_provider=lambda expected: Localization(
            node_id=expected,
            marker_id=expected,
            confidence=1.0,
            timestamp=step_clock(),
            source="fixed",
        ),
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
    )
    with pytest.raises(ConfigError) as excinfo:
        switcher.switch_edge(
            EdgeState.BLACK_LEFT,
            EdgeState.BLACK_RIGHT,
            authorization=authorization,
            location_id="B2",
        )
    assert "max_travel_m" in str(excinfo.value)
    # Refused before any bus write at all: the switch never started.
    assert transport.motor_payloads == []


# -- honest cleanup results --------------------------------------------------


def test_follower_reports_fault_when_the_final_stop_fails(step_clock, no_sleep) -> None:
    transport = MockI2CTransport(line_sensor_sequence=[LEFT] * 40)
    transport.fail_stops_after = 4  # stationary entry stop succeeds; final stop fails
    robot = Robot(transport, RobotConfig(), clock=step_clock, watchdog=False)
    robot.arm()
    follower = EdgeFollower(
        robot, config=FollowConfig(recovery_enabled=False), sleep=no_sleep
    )
    result = follower.follow_edge(EdgeState.BLACK_LEFT, max_iterations=2)
    assert result.stop_reason is FollowStopReason.FAULT
    assert robot.fault is not None


def test_switcher_reports_failure_when_the_final_stop_fails(step_clock, no_sleep) -> None:
    transport = MockI2CTransport(line_sensor_sequence=[LEFT] * 6 + [RIGHT] * 40)
    robot = Robot(transport, RobotConfig(), clock=step_clock, watchdog=False)
    robot.arm()
    switcher = EdgeSwitcher(
        robot,
        config=SwitchConfig(),
        clock=step_clock,
        sleep=no_sleep,
        location_provider=lambda expected: Localization(
            node_id=expected,
            marker_id=expected,
            confidence=1.0,
            timestamp=step_clock(),
            source="fixed",
        ),
    )
    transport.fail_stops_after = 0  # only the mandatory final stop fails
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
        destination_monitor=MockArrivalMonitor(
            polls_before_arrival=1, clock=step_clock
        ),
    )
    assert result.completed is False
    assert "final stop failed" in result.reason


def test_sensor_failure_is_not_relabelled_as_switch_timeout(step_clock, no_sleep) -> None:
    transport = MockI2CTransport(line_sensor_sequence=[LEFT] * 40)
    robot = Robot(transport, RobotConfig(), clock=step_clock, watchdog=False)
    robot.arm()
    switcher = EdgeSwitcher(
        robot,
        config=SwitchConfig(),
        clock=step_clock,
        sleep=no_sleep,
        location_provider=lambda expected: Localization(
            node_id=expected,
            marker_id=expected,
            confidence=1.0,
            timestamp=step_clock(),
            source="fixed",
        ),
    )
    transport.line_sensor_length = 2  # malformed frame -> SENSOR_FAILURE
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
        destination_monitor=MockArrivalMonitor(
            polls_before_arrival=1, clock=step_clock
        ),
    )
    assert result.completed is False
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.SENSOR_FAILURE
