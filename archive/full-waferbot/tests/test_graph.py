"""Track-map loading, validation, and physical-readiness rules."""

from __future__ import annotations

import copy
import json
from pathlib import Path

import pytest

from waferbot.errors import MapError
from waferbot.nav.graph import (
    EdgeAction,
    EdgeSide,
    NodeType,
    TrackMap,
)
from waferbot.sensing import EdgeState

EXAMPLE = Path("maps/example_track.json")


def example_dict() -> dict:
    return json.loads(EXAMPLE.read_text(encoding="utf-8"))


def test_example_map_loads_and_is_marked_illustrative():
    track = TrackMap.load(EXAMPLE)
    assert track.is_example is True
    assert track.physical_validated is False
    assert track.name.startswith("example-ring")
    assert len(track.nodes) == 21
    assert len(track.edges) == 28
    assert track.max_speed_mps == pytest.approx(0.15)
    assert track.adjacency()["A+"] == ["B+"]
    assert track.node("B2").node_type is NodeType.SWITCH
    assert {edge.action for edge in track.edges} == {
        EdgeAction.FOLLOW_EDGE,
        EdgeAction.SWITCH_EDGE,
        EdgeAction.TURN,
        EdgeAction.DOCK,
        EdgeAction.STOP,
    }


def test_edge_side_mapping_is_not_implied_by_node_names():
    track = TrackMap.load(EXAMPLE)
    assert track.edge_state_for(EdgeSide.POSITIVE, where="test") is EdgeState.BLACK_LEFT
    assert track.edge_state_for(EdgeSide.NEGATIVE, where="test") is EdgeState.BLACK_RIGHT

    data = example_dict()
    data["edge_side_map"]["positive"] = None
    uncalibrated = TrackMap.from_dict(data)
    with pytest.raises(MapError):
        uncalibrated.edge_state_for(EdgeSide.POSITIVE, where="test")


def test_physical_readiness_requires_measured_flagged_map():
    track = TrackMap.load(EXAMPLE)
    with pytest.raises(MapError):
        track.require_physical_ready()

    data = example_dict()
    data["is_example"] = False
    data["physical_validated"] = True
    ready = TrackMap.from_dict(data)
    ready.require_physical_ready()

    data["edge_side_map"]["negative"] = None
    with pytest.raises(MapError):
        TrackMap.from_dict(data).require_physical_ready()


@pytest.mark.parametrize(
    ("mutate", "message"),
    [
        (lambda data: data["edges"][0].update(destination="ZZ"), "not a node"),
        (lambda data: data["nodes"].append(copy.deepcopy(data["nodes"][0])), "duplicate node"),
        (lambda data: data["edges"].append(copy.deepcopy(data["edges"][0])), "duplicate edge"),
        (lambda data: data["edges"][0].update(distance_m=float("nan")), "must be finite"),
        (lambda data: data["edges"][0].update(distance_m=-1.0), "must be >= 0"),
        (lambda data: data["edges"][0].update(speed_limit_mps=0.0), "speed_limit_mps"),
        (lambda data: data["edges"][0].update(speed_limit_mps=9.0), "speed_limit_mps"),
        (lambda data: data["edges"][0].update(action="TELEPORT"), "edge.action"),
        (lambda data: data["edges"][0].update(enabled="yes"), "enabled"),
        (lambda data: data["nodes"][0].update(x=float("inf")), "must be finite"),
        (lambda data: data["nodes"][0].update(node_id=""), "non-empty node_id"),
        (lambda data: data["nodes"][0].update(unexpected=1), "unknown node keys"),
        (lambda data: data["edges"][0].update(unexpected=1), "unknown edge keys"),
        (lambda data: data.update(unexpected=1), "unknown track map keys"),
        (lambda data: data["edges"][0].update(crossing_angle_deg=10.0), "carries crossing_angle"),
    ],
)
def test_invalid_maps_are_rejected(mutate, message):
    data = example_dict()
    mutate(data)
    with pytest.raises(MapError) as excinfo:
        TrackMap.from_dict(data)
    assert message in str(excinfo.value)


def test_switch_edges_need_an_angle_and_a_side_change():
    data = example_dict()
    switch = next(e for e in data["edges"] if e["action"] == "SWITCH_EDGE")
    switch["crossing_angle_deg"] = None
    with pytest.raises(MapError):
        TrackMap.from_dict(data)

    data = example_dict()
    switch = next(e for e in data["edges"] if e["action"] == "SWITCH_EDGE")
    switch["crossing_angle_deg"] = 120.0
    with pytest.raises(MapError):
        TrackMap.from_dict(data)

    data = example_dict()
    switch = next(e for e in data["edges"] if e["action"] == "SWITCH_EDGE")
    switch["target_edge_side"] = switch["edge_side"]
    with pytest.raises(MapError) as excinfo:
        TrackMap.from_dict(data)
    assert "does not change edge side" in str(excinfo.value)


def test_follow_edge_must_not_change_side():
    data = example_dict()
    follow = next(e for e in data["edges"] if e["action"] == "FOLLOW_EDGE")
    follow["edge_side"] = "positive"
    follow["target_edge_side"] = "negative"
    with pytest.raises(MapError) as excinfo:
        TrackMap.from_dict(data)
    assert "use SWITCH_EDGE" in str(excinfo.value)


def test_turn_and_dock_need_durations():
    data = example_dict()
    turn = next(e for e in data["edges"] if e["action"] == "TURN")
    turn["turn_time_s"] = 0.0
    with pytest.raises(MapError):
        TrackMap.from_dict(data)

    data = example_dict()
    dock = next(e for e in data["edges"] if e["action"] == "DOCK")
    dock["dock_time_s"] = 0.0
    with pytest.raises(MapError):
        TrackMap.from_dict(data)


def test_disabled_edges_are_excluded_from_neighbours():
    data = example_dict()
    data["edges"][0]["enabled"] = False
    track = TrackMap.from_dict(data)
    assert track.adjacency()["A+"] == []
    assert track.adjacency(include_disabled=True)["A+"] == ["B+"]


def test_map_round_trip(tmp_path):
    track = TrackMap.load(EXAMPLE)
    path = tmp_path / "track.json"
    track.save(path)
    reloaded = TrackMap.load(path)
    assert reloaded.to_dict() == track.to_dict()


def test_missing_map_is_a_map_error(tmp_path):
    with pytest.raises(MapError):
        TrackMap.load(tmp_path / "absent.json")


def test_malformed_map_is_a_map_error(tmp_path):
    path = tmp_path / "broken.json"
    path.write_text("{oops")
    with pytest.raises(MapError):
        TrackMap.load(path)


def test_unknown_node_lookup():
    track = TrackMap.load(EXAMPLE)
    assert track.has_node("B2") is True
    assert track.has_node("nope") is False
    with pytest.raises(MapError):
        track.node("nope")

