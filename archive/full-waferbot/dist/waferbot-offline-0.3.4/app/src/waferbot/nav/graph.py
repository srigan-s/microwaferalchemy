"""Directed track graph loaded from JSON.

The graph is data: editing the map file never requires touching navigation code.
Node labels are free-form; the ``+``/``-`` suffix in the example map is only a
naming convention for the two tape edges. ``edge_side`` means "which tape edge
this node/edge runs along" and is mapped to a physical edge state
(``BLACK_LEFT`` / ``BLACK_RIGHT``) by ``edge_side_map``.

``edge_side_map`` values are ``null`` until an operator measures which physical
edge each side corresponds to. Physical execution refuses to run while any
mapping is unset: positive/negative labels never imply a sensor polarity.
"""

from __future__ import annotations

import json
import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any

from ..errors import MapError
from ..sensing.edge import EdgeState

#: Sanity cap that catches unit mistakes (for example cm entered as m).
MAX_SPEED_LIMIT_MPS = 5.0


class NodeType(str, Enum):
    STATION = "station"
    JUNCTION = "junction"
    SWITCH = "switch"
    DOCK = "dock"
    INTERMEDIATE = "intermediate"


class EdgeAction(str, Enum):
    FOLLOW_EDGE = "FOLLOW_EDGE"
    SWITCH_EDGE = "SWITCH_EDGE"
    TURN = "TURN"
    DOCK = "DOCK"
    STOP = "STOP"


class EdgeSide(str, Enum):
    POSITIVE = "positive"
    NEGATIVE = "negative"


class TravelDirection(str, Enum):
    FORWARD = "forward"
    REVERSE = "reverse"


@dataclass(frozen=True)
class MapNode:
    node_id: str
    station_id: str
    x: float
    y: float
    heading: float
    edge_side: EdgeSide | None
    direction: TravelDirection | None
    node_type: NodeType
    marker_id: str | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "node_id": self.node_id,
            "station_id": self.station_id,
            "x": self.x,
            "y": self.y,
            "heading": self.heading,
            "edge_side": self.edge_side.value if self.edge_side else None,
            "direction": self.direction.value if self.direction else None,
            "node_type": self.node_type.value,
            "marker_id": self.marker_id,
        }


@dataclass(frozen=True)
class MapEdge:
    source: str
    destination: str
    distance_m: float
    speed_limit_mps: float
    action: EdgeAction
    crossing_angle_deg: float | None = None
    enabled: bool = True
    edge_side: EdgeSide | None = None
    target_edge_side: EdgeSide | None = None
    direction: TravelDirection | None = None
    switch_time_s: float = 0.0
    dock_time_s: float = 0.0
    turn_time_s: float = 0.0
    notes: str = ""

    @property
    def overhead_s(self) -> float:
        """Switch/dock/turn overhead added to the travel time."""
        return self.switch_time_s + self.dock_time_s + self.turn_time_s

    def travel_time_s(self) -> float:
        if self.action is EdgeAction.STOP:
            return 0.0
        return self.distance_m / self.speed_limit_mps

    def cost_s(self) -> float:
        return self.travel_time_s() + self.overhead_s

    def as_dict(self) -> dict[str, Any]:
        return {
            "source": self.source,
            "destination": self.destination,
            "distance_m": self.distance_m,
            "speed_limit_mps": self.speed_limit_mps,
            "action": self.action.value,
            "crossing_angle_deg": self.crossing_angle_deg,
            "enabled": self.enabled,
            "edge_side": self.edge_side.value if self.edge_side else None,
            "target_edge_side": (
                self.target_edge_side.value if self.target_edge_side else None
            ),
            "direction": self.direction.value if self.direction else None,
            "switch_time_s": self.switch_time_s,
            "dock_time_s": self.dock_time_s,
            "turn_time_s": self.turn_time_s,
            "notes": self.notes,
        }


@dataclass(frozen=True)
class TrackMap:
    name: str
    is_example: bool
    physical_validated: bool
    edge_side_map: Mapping[str, str | None]
    nodes: tuple[MapNode, ...]
    edges: tuple[MapEdge, ...]
    notes: str = ""
    schema_version: int = 1

    # -- construction -------------------------------------------------------

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "TrackMap":
        if not isinstance(data, Mapping):
            raise MapError("track map must be a JSON object")
        known = {
            "schema_version",
            "name",
            "is_example",
            "physical_validated",
            "edge_side_map",
            "nodes",
            "edges",
            "notes",
        }
        unknown = set(data) - known
        if unknown:
            raise MapError(f"unknown track map keys: {sorted(unknown)}")
        for required in ("name", "nodes", "edges"):
            if required not in data:
                raise MapError(f"track map is missing required key {required!r}")
        try:
            nodes = tuple(_node_from_dict(entry) for entry in data["nodes"])
            edges = tuple(_edge_from_dict(entry) for entry in data["edges"])
        except TypeError as exc:
            raise MapError(f"nodes and edges must be lists: {exc}") from exc
        for flag in ("is_example", "physical_validated"):
            value = data.get(flag, False)
            if not isinstance(value, bool):
                raise MapError(f"track map {flag} must be a boolean")
        side_map = data.get("edge_side_map", {"positive": None, "negative": None})
        if not isinstance(side_map, Mapping):
            raise MapError("edge_side_map must be an object")
        track = cls(
            schema_version=int(data.get("schema_version", 1)),
            name=str(data["name"]),
            is_example=bool(data.get("is_example", False)),
            physical_validated=bool(data.get("physical_validated", False)),
            edge_side_map=dict(side_map),
            nodes=nodes,
            edges=edges,
            notes=str(data.get("notes", "")),
        )
        return track.validate()

    @classmethod
    def load(cls, path: str | Path) -> "TrackMap":
        path = Path(path)
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise MapError(f"track map not found: {path}") from exc
        except json.JSONDecodeError as exc:
            raise MapError(f"track map {path} is not valid JSON: {exc}") from exc
        return cls.from_dict(raw)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "name": self.name,
            "is_example": self.is_example,
            "physical_validated": self.physical_validated,
            "edge_side_map": dict(self.edge_side_map),
            "notes": self.notes,
            "nodes": [node.as_dict() for node in self.nodes],
            "edges": [edge.as_dict() for edge in self.edges],
        }

    def save(self, path: str | Path) -> None:
        Path(path).write_text(
            json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8"
        )

    # -- validation ---------------------------------------------------------

    def validate(self) -> "TrackMap":
        if self.schema_version != 1:
            raise MapError(
                f"unsupported track map schema_version {self.schema_version}"
            )
        if not self.name:
            raise MapError("track map needs a name")
        seen: set[str] = set()
        for node in self.nodes:
            if not node.node_id or not isinstance(node.node_id, str):
                raise MapError("every node needs a non-empty node_id")
            if node.node_id in seen:
                raise MapError(f"duplicate node id {node.node_id!r}")
            seen.add(node.node_id)
            for name in ("x", "y", "heading"):
                value = getattr(node, name)
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise MapError(
                        f"node {node.node_id} field {name} must be a number"
                    )
                if not math.isfinite(value):
                    raise MapError(
                        f"node {node.node_id} field {name} must be finite"
                    )
        if not self.nodes:
            raise MapError("track map has no nodes")

        by_id = {node.node_id: node for node in self.nodes}
        seen_edges: set[tuple[str, str, str]] = set()
        for edge in self.edges:
            if edge.source not in by_id:
                raise MapError(f"edge source {edge.source!r} is not a node")
            if edge.destination not in by_id:
                raise MapError(
                    f"edge destination {edge.destination!r} is not a node"
                )
            key = (edge.source, edge.destination, edge.action.value)
            if key in seen_edges:
                raise MapError(
                    "duplicate edge "
                    f"{edge.source}->{edge.destination} ({edge.action.value})"
                )
            seen_edges.add(key)
            for name in ("distance_m", "speed_limit_mps", "switch_time_s", "dock_time_s", "turn_time_s"):
                value = getattr(edge, name)
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise MapError(
                        f"edge {edge.source}->{edge.destination} field {name} "
                        "must be a number"
                    )
                if not math.isfinite(value):
                    raise MapError(
                        f"edge {edge.source}->{edge.destination} field {name} "
                        "must be finite"
                    )
                if value < 0:
                    raise MapError(
                        f"edge {edge.source}->{edge.destination} field {name} "
                        "must be >= 0"
                    )
            if not 0 < edge.speed_limit_mps <= MAX_SPEED_LIMIT_MPS:
                raise MapError(
                    f"edge {edge.source}->{edge.destination} speed_limit_mps "
                    f"must be in (0, {MAX_SPEED_LIMIT_MPS}]"
                )
            if not isinstance(edge.enabled, bool):
                raise MapError(
                    f"edge {edge.source}->{edge.destination} enabled must be a bool"
                )
            if edge.action in (EdgeAction.FOLLOW_EDGE, EdgeAction.SWITCH_EDGE):
                if edge.distance_m <= 0:
                    raise MapError(
                        f"edge {edge.source}->{edge.destination} "
                        f"{edge.action.value} needs distance_m > 0"
                    )
            if edge.action is EdgeAction.SWITCH_EDGE:
                if edge.crossing_angle_deg is None:
                    raise MapError(
                        f"switch edge {edge.source}->{edge.destination} needs "
                        "crossing_angle_deg"
                    )
                angle = edge.crossing_angle_deg
                if (
                    isinstance(angle, bool)
                    or not isinstance(angle, (int, float))
                    or not math.isfinite(angle)
                    or not 0 < angle <= 90
                ):
                    raise MapError(
                        f"switch edge {edge.source}->{edge.destination} "
                        "crossing_angle_deg must be finite and in (0, 90]; 90 is "
                        "a lateral crossing, smaller values are diagonal"
                    )
                target_side = self.edge_target_side(edge)
                if edge.edge_side is not None and target_side is not None:
                    if target_side is edge.edge_side:
                        raise MapError(
                            f"switch edge {edge.source}->{edge.destination} does "
                            "not change edge side"
                        )
            elif edge.crossing_angle_deg is not None:
                raise MapError(
                    f"edge {edge.source}->{edge.destination} is "
                    f"{edge.action.value} but carries crossing_angle_deg"
                )
            if edge.action is EdgeAction.FOLLOW_EDGE:
                target_side = self.edge_target_side(edge)
                if (
                    edge.edge_side is not None
                    and target_side is not None
                    and target_side is not edge.edge_side
                ):
                    raise MapError(
                        f"FOLLOW_EDGE {edge.source}->{edge.destination} changes "
                        "edge side; use SWITCH_EDGE"
                    )
                if edge.target_edge_side is not None:
                    raise MapError(
                        f"FOLLOW_EDGE {edge.source}->{edge.destination} must not "
                        "declare target_edge_side"
                    )
            if edge.action is EdgeAction.TURN and edge.turn_time_s <= 0:
                raise MapError(
                    f"TURN edge {edge.source}->{edge.destination} needs "
                    "turn_time_s > 0"
                )
            if (
                edge.action is EdgeAction.TURN
                and edge.direction is TravelDirection.REVERSE
            ):
                raise MapError(
                    f"TURN edge {edge.source}->{edge.destination} declares "
                    "direction=reverse; TURN uses the absolute map heading delta "
                    "and reverse is not supported"
                )
            if edge.action is EdgeAction.DOCK and edge.dock_time_s <= 0:
                raise MapError(
                    f"DOCK edge {edge.source}->{edge.destination} needs "
                    "dock_time_s > 0"
                )
            if edge.action is EdgeAction.STOP:
                source_node = by_id[edge.source]
                destination_node = by_id[edge.destination]
                if edge.source != edge.destination:
                    gap = math.hypot(
                        destination_node.x - source_node.x,
                        destination_node.y - source_node.y,
                    )
                    if gap > 1e-6:
                        raise MapError(
                            f"STOP edge {edge.source}->{edge.destination} would be a "
                            "spatial teleport; STOP must be a stationary, "
                            "self/coincident action"
                        )

            # Edge-side and direction conventions must be consistent with the
            # nodes so a FOLLOW step cannot silently start on another side.
            source_node = by_id[edge.source]
            destination_node = by_id[edge.destination]
            if edge.edge_side is not None and source_node.edge_side is not None:
                if source_node.edge_side is not edge.edge_side:
                    raise MapError(
                        f"edge {edge.source}->{edge.destination} declares "
                        f"edge_side {edge.edge_side.value} but source node "
                        f"{edge.source} is on {source_node.edge_side.value}"
                    )
            if edge.action is EdgeAction.SWITCH_EDGE:
                target_side = self.edge_target_side(edge)
                if (
                    target_side is not None
                    and destination_node.edge_side is not None
                    and destination_node.edge_side is not target_side
                ):
                    raise MapError(
                        f"switch edge {edge.source}->{edge.destination} targets "
                        f"{target_side.value} but destination node is on "
                        f"{destination_node.edge_side.value}"
                    )
            if (
                edge.direction is not None
                and destination_node.direction is not None
                and edge.direction is not destination_node.direction
            ):
                raise MapError(
                    f"edge {edge.source}->{edge.destination} declares direction "
                    f"{edge.direction.value} but destination node "
                    f"{edge.destination} is {destination_node.direction.value}"
                )

        for key, value in self.edge_side_map.items():
            if key not in {side.value for side in EdgeSide}:
                raise MapError(
                    f"edge_side_map key {key!r} must be 'positive' or 'negative'"
                )
            if value is not None:
                try:
                    EdgeState.from_name(value)
                except ValueError as exc:
                    raise MapError(str(exc)) from exc
                if EdgeState.from_name(value) not in (
                    EdgeState.BLACK_LEFT,
                    EdgeState.BLACK_RIGHT,
                ):
                    raise MapError(
                        f"edge_side_map[{key!r}] must be BLACK_LEFT or BLACK_RIGHT"
                    )
        return self

    def require_physical_ready(self, *, allow_example: bool = False) -> None:
        """Refuse physical execution until the map is real and mapped."""
        if self.is_example and not allow_example:
            raise MapError(
                f"track map {self.name!r} is an illustrative example; measure the "
                "real topology, set is_example=false, and acknowledge it before "
                "physical execution"
            )
        if not self.physical_validated:
            raise MapError(
                f"track map {self.name!r} is not marked physical_validated"
            )
        missing = [
            side.value
            for side in EdgeSide
            if not self.edge_side_map.get(side.value)
        ]
        if missing:
            raise MapError(
                "edge_side_map is not calibrated for "
                f"{', '.join(missing)}; record which physical edge each side is"
            )

    # -- lookup -------------------------------------------------------------

    def node(self, node_id: str) -> MapNode:
        for node in self.nodes:
            if node.node_id == node_id:
                return node
        raise MapError(f"unknown node {node_id!r}")

    def has_node(self, node_id: str) -> bool:
        return any(node.node_id == node_id for node in self.nodes)

    def neighbours(self, node_id: str, *, include_disabled: bool = False) -> tuple[MapEdge, ...]:
        edges = self._adjacency_index().get(node_id, ())
        if include_disabled:
            return tuple(edges)
        return tuple(edge for edge in edges if edge.enabled)

    def adjacency(self, *, include_disabled: bool = False) -> dict[str, list[str]]:
        return {
            node.node_id: [
                edge.destination
                for edge in self.neighbours(node.node_id, include_disabled=include_disabled)
            ]
            for node in self.nodes
        }

    def _adjacency_index(self) -> dict[str, tuple[MapEdge, ...]]:
        """Real adjacency list: node id -> outgoing edges (built once)."""
        cached = getattr(self, "_adjacency_cache", None)
        if cached is None:
            index: dict[str, list[MapEdge]] = {node.node_id: [] for node in self.nodes}
            for edge in self.edges:
                index.setdefault(edge.source, []).append(edge)
            cached = {key: tuple(value) for key, value in index.items()}
            object.__setattr__(self, "_adjacency_cache", cached)
        return cached

    def edge_matching(self, edge: MapEdge) -> MapEdge | None:
        """Return the map's own copy of ``edge`` when it exists and matches."""
        for candidate in self._adjacency_index().get(edge.source, ()):
            if (
                candidate.destination == edge.destination
                and candidate.action is edge.action
            ):
                return candidate
        return None

    def marker_to_node(self) -> dict[str, str]:
        """Marker id -> node id, for localizers that report marker ids only."""
        return {
            node.marker_id: node.node_id
            for node in self.nodes
            if node.marker_id
        }

    @property
    def enabled_edges(self) -> tuple[MapEdge, ...]:
        return tuple(edge for edge in self.edges if edge.enabled)

    @property
    def max_speed_mps(self) -> float:
        speeds = [edge.speed_limit_mps for edge in self.enabled_edges]
        return max(speeds) if speeds else 0.0

    def edge_target_side(self, edge: MapEdge) -> EdgeSide | None:
        """Edge side on the destination end, from the edge or the node."""
        if edge.target_edge_side is not None:
            return edge.target_edge_side
        node = self.node(edge.destination)
        return node.edge_side

    def edge_state_for(self, side: EdgeSide | None, *, where: str) -> EdgeState:
        """Map a positive/negative side to a physical edge state."""
        if side is None:
            raise MapError(f"{where}: no edge_side recorded")
        mapped = self.edge_side_map.get(side.value)
        if not mapped:
            raise MapError(
                f"{where}: edge_side_map[{side.value!r}] is not calibrated; "
                "positive/negative to BLACK_LEFT/BLACK_RIGHT must be measured"
            )
        return EdgeState.from_name(mapped)


def _node_from_dict(data: Mapping[str, Any]) -> MapNode:
    if not isinstance(data, Mapping):
        raise MapError("every node must be an object")
    known = {
        "node_id",
        "station_id",
        "x",
        "y",
        "heading",
        "edge_side",
        "direction",
        "node_type",
        "marker_id",
    }
    unknown = set(data) - known
    if unknown:
        raise MapError(f"unknown node keys: {sorted(unknown)}")
    for required in ("node_id", "station_id", "x", "y", "heading", "node_type"):
        if required not in data:
            raise MapError(f"node is missing required key {required!r}")
    return MapNode(
        node_id=str(data["node_id"]),
        station_id=str(data["station_id"]),
        x=data["x"],
        y=data["y"],
        heading=data["heading"],
        edge_side=_enum_or_none(EdgeSide, data.get("edge_side"), "node.edge_side"),
        direction=_enum_or_none(
            TravelDirection, data.get("direction"), "node.direction"
        ),
        node_type=_enum_or_none(NodeType, data["node_type"], "node.node_type", required=True),
        marker_id=None if data.get("marker_id") is None else str(data["marker_id"]),
    )


def _edge_from_dict(data: Mapping[str, Any]) -> MapEdge:
    if not isinstance(data, Mapping):
        raise MapError("every edge must be an object")
    known = {
        "source",
        "destination",
        "distance_m",
        "speed_limit_mps",
        "action",
        "crossing_angle_deg",
        "enabled",
        "edge_side",
        "target_edge_side",
        "direction",
        "switch_time_s",
        "dock_time_s",
        "turn_time_s",
        "notes",
    }
    unknown = set(data) - known
    if unknown:
        raise MapError(f"unknown edge keys: {sorted(unknown)}")
    for required in ("source", "destination", "distance_m", "speed_limit_mps", "action"):
        if required not in data:
            raise MapError(f"edge is missing required key {required!r}")
    action = _enum_or_none(EdgeAction, data["action"], "edge.action", required=True)
    enabled = data.get("enabled", True)
    if not isinstance(enabled, bool):
        raise MapError("edge.enabled must be a boolean")
    return MapEdge(
        source=str(data["source"]),
        destination=str(data["destination"]),
        distance_m=data["distance_m"],
        speed_limit_mps=data["speed_limit_mps"],
        action=action,
        crossing_angle_deg=data.get("crossing_angle_deg"),
        enabled=enabled,
        edge_side=_enum_or_none(EdgeSide, data.get("edge_side"), "edge.edge_side"),
        target_edge_side=_enum_or_none(
            EdgeSide, data.get("target_edge_side"), "edge.target_edge_side"
        ),
        direction=_enum_or_none(
            TravelDirection, data.get("direction"), "edge.direction"
        ),
        switch_time_s=data.get("switch_time_s", 0.0),
        dock_time_s=data.get("dock_time_s", 0.0),
        turn_time_s=data.get("turn_time_s", 0.0),
        notes=str(data.get("notes", "")),
    )


def _enum_or_none(enum_cls, value, where: str, *, required: bool = False):
    if value is None:
        if required:
            raise MapError(f"{where} is required")
        return None
    try:
        return enum_cls(str(value))
    except ValueError as exc:
        allowed = ", ".join(member.value for member in enum_cls)
        raise MapError(f"{where} must be one of {allowed}; got {value!r}") from exc


__all__ = [
    "EdgeAction",
    "EdgeSide",
    "MAX_SPEED_LIMIT_MPS",
    "MapEdge",
    "MapNode",
    "NodeType",
    "TrackMap",
    "TravelDirection",
]
