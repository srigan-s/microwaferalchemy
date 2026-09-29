"""Dijkstra and A* planning, waypoint ordering, and heuristic safety."""

from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest

from waferbot.errors import PlanningError
from waferbot.nav.graph import EdgeAction, TrackMap
from waferbot.nav.planner import Planner, plan_route

EXAMPLE = Path("maps/example_track.json")


def example_dict() -> dict:
    return json.loads(EXAMPLE.read_text(encoding="utf-8"))


def test_dijkstra_route_with_required_waypoint():
    track = TrackMap.load(EXAMPLE)
    route = plan_route(track, "A+", "D-", ["B-"])
    assert route.nodes == ("A+", "B+", "B1", "B2", "B3", "B-", "C-", "D-")
    assert EdgeAction.SWITCH_EDGE in route.actions
    assert route.required_waypoints == ("B-",)
    assert "B-" in route.nodes
    assert route.nodes.index("B-") > 0
    assert route.nodes.index("B-") < len(route.nodes) - 1
    assert route.total_cost_s == pytest.approx(
        sum(step.cost_s for step in route.steps)
    )


def test_cost_model_is_travel_plus_switch_plus_dock():
    track = TrackMap.load(EXAMPLE)
    planner = Planner(track)
    route = planner.plan_route("A+", "B-")
    switch_step = next(step for step in route.steps if step.action is EdgeAction.SWITCH_EDGE)
    assert switch_step.overhead_s == pytest.approx(2.0)
    assert switch_step.travel_time_s == pytest.approx(0.2 / 0.1)
    assert switch_step.cost_s == pytest.approx(2.0 + 2.0)

    dock = next(
        edge for edge in track.edges if edge.action is EdgeAction.DOCK
    )
    assert dock.cost_s() == pytest.approx(0.8 / 0.08 + 1.5)


def test_astar_matches_dijkstra_when_the_metric_is_consistent():
    track = TrackMap.load(EXAMPLE)
    dijkstra = Planner(track, algorithm="dijkstra")
    astar = Planner(track, algorithm="astar")
    assert astar.metric_consistent() is True
    assert astar.admissible_heuristic("D-", "A+") > 0
    route_d = dijkstra.plan_route("A+", "D-", ["B-"])
    route_a = astar.plan_route("A+", "D-", ["B-"])
    assert route_a.nodes == route_d.nodes
    assert route_a.total_cost_s == pytest.approx(route_d.total_cost_s)
    assert route_a.heuristic_admissible is True


def test_astar_drops_to_zero_when_the_map_is_not_metric():
    data = example_dict()
    # Declare a long edge as much shorter than the straight line: the heuristic
    # must not be trusted any more.
    edge = next(e for e in data["edges"] if e["source"] == "A+" and e["destination"] == "B+")
    edge["distance_m"] = 0.5
    track = TrackMap.from_dict(data)
    astar = Planner(track, algorithm="astar")
    assert astar.metric_consistent() is False
    assert astar.admissible_heuristic("D-", "A+") == 0.0
    route = astar.plan_route("A+", "D-", ["B-"])
    assert route.heuristic_admissible is False
    assert route.nodes[0] == "A+"


def test_route_from_start_to_itself_is_empty():
    track = TrackMap.load(EXAMPLE)
    route = plan_route(track, "A+", "A+")
    assert route.steps == ()
    assert route.nodes == ("A+",)
    assert route.total_cost_s == 0.0


def test_unknown_nodes_are_planning_errors():
    track = TrackMap.load(EXAMPLE)
    with pytest.raises(PlanningError):
        plan_route(track, "nope", "D-")
    with pytest.raises(PlanningError):
        plan_route(track, "A+", "nope")
    with pytest.raises(PlanningError):
        plan_route(track, "A+", "D-", ["nope"])


def test_disabled_edges_are_respected():
    data = example_dict()
    data["edges"][0]["enabled"] = False  # A+ -> B+
    track = TrackMap.from_dict(data)
    with pytest.raises(PlanningError):
        plan_route(track, "A+", "B+")


def test_unreachable_destination_is_a_planning_error():
    data = example_dict()
    data["edges"] = [
        edge
        for edge in data["edges"]
        if not (edge["source"] == "A+" and edge["destination"] == "B+")
    ]
    track = TrackMap.from_dict(data)
    with pytest.raises(PlanningError):
        plan_route(track, "A+", "D-")


def test_mandatory_waypoints_must_stay_executable():
    track = TrackMap.load(EXAMPLE)
    route = plan_route(track, "B-", "D-", ["A+"])
    # A+ is unreachable from the inner lane without a switch, so the planner
    # must find a real path through the graph rather than skipping the waypoint.
    assert "A+" in route.nodes
    assert route.nodes[-1] == "D-"


def test_algorithm_argument_is_validated():
    track = TrackMap.load(EXAMPLE)
    with pytest.raises(ValueError):
        Planner(track, algorithm="floyd")
    with pytest.raises(ValueError):
        Planner(track).plan_route("A+", "D-", [], algorithm="bellman")


def test_route_serialises_for_the_cli():
    track = TrackMap.load(EXAMPLE)
    payload = plan_route(track, "A+", "B-").as_dict()
    assert payload["start_node"] == "A+"
    assert payload["destination_node"] == "B-"
    assert isinstance(payload["steps"], list)
    assert payload["steps"][0]["action"] == "FOLLOW_EDGE"
    assert payload["route_id"].startswith("route-")


def test_waypoint_duplicates_do_not_add_empty_legs():
    track = TrackMap.load(EXAMPLE)
    route = plan_route(track, "A+", "B-", ["A+", "B-", "B-"])
    assert route.nodes[0] == "A+"
    assert route.nodes[-1] == "B-"
    assert route.total_cost_s > 0

