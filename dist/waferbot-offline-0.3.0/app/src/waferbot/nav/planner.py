"""Dijkstra (default) and A* route planning over the track graph.

Cost model::

    cost(edge) = distance_m / speed_limit_mps + switch_time_s + dock_time_s
                 + turn_time_s

which is exactly the requested ``travel_time + switching_time + docking_time``.

The A* heuristic is ``euclidean(node, goal) / max_speed`` and is only used when
the map is metrically consistent: every enabled edge must satisfy
``euclidean(u, v) <= distance_m``. If any edge declares a distance shorter than
the straight line between its endpoints, the heuristic drops to zero (making A*
behave as Dijkstra) instead of risking an inadmissible search.
"""

from __future__ import annotations

import hashlib
import heapq
import math
from dataclasses import dataclass
from typing import Any

from ..errors import PlanningError
from .graph import EdgeAction, MapEdge, TrackMap

ALGORITHMS = ("dijkstra", "astar")


@dataclass(frozen=True)
class RouteStep:
    index: int
    edge: MapEdge
    travel_time_s: float
    overhead_s: float
    cost_s: float

    @property
    def source(self) -> str:
        return self.edge.source

    @property
    def destination(self) -> str:
        return self.edge.destination

    @property
    def action(self) -> EdgeAction:
        return self.edge.action

    def as_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "source": self.source,
            "destination": self.destination,
            "action": self.action.value,
            "distance_m": self.edge.distance_m,
            "speed_limit_mps": self.edge.speed_limit_mps,
            "crossing_angle_deg": self.edge.crossing_angle_deg,
            "edge_side": self.edge.edge_side.value if self.edge.edge_side else None,
            "target_edge_side": (
                self.edge.target_edge_side.value
                if self.edge.target_edge_side
                else None
            ),
            "travel_time_s": self.travel_time_s,
            "overhead_s": self.overhead_s,
            "cost_s": self.cost_s,
        }


@dataclass(frozen=True)
class Route:
    route_id: str
    start_node: str
    destination_node: str
    required_waypoints: tuple[str, ...]
    algorithm: str
    nodes: tuple[str, ...]
    steps: tuple[RouteStep, ...]
    total_cost_s: float
    heuristic_admissible: bool = True

    @property
    def edges(self) -> tuple[MapEdge, ...]:
        return tuple(step.edge for step in self.steps)

    @property
    def actions(self) -> tuple[EdgeAction, ...]:
        return tuple(step.action for step in self.steps)

    @property
    def distance_m(self) -> float:
        return sum(step.edge.distance_m for step in self.steps)

    def describe(self) -> str:
        hops = " -> ".join(self.nodes)
        return (
            f"route {self.route_id} ({self.algorithm}): {hops} "
            f"[{self.distance_m:.2f} m, {self.total_cost_s:.1f} s]"
        )

    def as_dict(self) -> dict[str, Any]:
        return {
            "route_id": self.route_id,
            "start_node": self.start_node,
            "destination_node": self.destination_node,
            "required_waypoints": list(self.required_waypoints),
            "algorithm": self.algorithm,
            "nodes": list(self.nodes),
            "distance_m": self.distance_m,
            "total_cost_s": self.total_cost_s,
            "heuristic_admissible": self.heuristic_admissible,
            "steps": [step.as_dict() for step in self.steps],
        }


class Planner:
    """Plans routes over a validated :class:`~waferbot.nav.graph.TrackMap`."""

    def __init__(self, track: TrackMap, *, algorithm: str = "dijkstra") -> None:
        if algorithm not in ALGORITHMS:
            raise ValueError(f"unknown algorithm {algorithm!r}")
        self.track = track.validate()
        self.algorithm = algorithm
        self._coordinates = {
            node.node_id: (node.x, node.y) for node in self.track.nodes
        }

    # -- heuristic ----------------------------------------------------------

    def heuristic_scale(self) -> float:
        """1/max_speed, or 0 when a heuristic would be unsafe."""
        max_speed = self.track.max_speed_mps
        if max_speed <= 0:
            return 0.0
        return 1.0 / max_speed

    def metric_consistent(self) -> bool:
        """True when no enabled edge is shorter than its straight-line span."""
        for edge in self.track.enabled_edges:
            ax, ay = self._coordinates[edge.source]
            bx, by = self._coordinates[edge.destination]
            # STOP edges are checked too: a zero-cost jump between distinct
            # coordinates would otherwise let a teleport break admissibility.
            if math.hypot(bx - ax, by - ay) > edge.distance_m + 1e-9:
                return False
        return True

    def admissible_heuristic(self, goal: str, node: str) -> float:
        scale = self.heuristic_scale()
        if scale == 0.0 or not self.metric_consistent():
            return 0.0
        ax, ay = self._coordinates[node]
        bx, by = self._coordinates[goal]
        return math.hypot(bx - ax, by - ay) * scale

    # -- search -------------------------------------------------------------

    def search(
        self, source: str, destination: str, *, algorithm: str | None = None
    ) -> tuple[list[MapEdge], float]:
        algorithm = algorithm or self.algorithm
        if algorithm not in ALGORITHMS:
            raise ValueError(f"unknown algorithm {algorithm!r}")
        for node_id in (source, destination):
            if not self.track.has_node(node_id):
                raise PlanningError(f"unknown node {node_id!r}")
        if source == destination:
            return [], 0.0

        counter = 0
        best: dict[str, float] = {source: 0.0}
        came_from: dict[str, MapEdge] = {}
        frontier: list[tuple[float, int, str]] = [
            (self._heuristic(source, destination, algorithm), counter, source)
        ]
        settled: set[str] = set()

        while frontier:
            _estimate, _tie, node = heapq.heappop(frontier)
            if node in settled:
                continue
            if node == destination:
                return self._reconstruct(came_from, source, destination, best)
            settled.add(node)
            for edge in self.track.neighbours(node):
                cost = edge.cost_s()
                if cost < 0 or not math.isfinite(cost):
                    raise PlanningError(
                        f"edge {edge.source}->{edge.destination} has a non-finite cost"
                    )
                candidate = best[node] + cost
                if candidate < best.get(edge.destination, math.inf) - 1e-12:
                    best[edge.destination] = candidate
                    came_from[edge.destination] = edge
                    counter += 1
                    heapq.heappush(
                        frontier,
                        (
                            candidate
                            + self._heuristic(edge.destination, destination, algorithm),
                            counter,
                            edge.destination,
                        ),
                    )
        raise PlanningError(
            f"no enabled route from {source!r} to {destination!r}"
        )

    def shortest_path(
        self, source: str, destination: str, *, algorithm: str | None = None
    ) -> list[MapEdge]:
        edges, _cost = self.search(source, destination, algorithm=algorithm)
        return edges

    def _priority(
        self, node: str, goal: str, cost: float, algorithm: str
    ) -> float:
        """Deprecated shim kept for callers: priority is ``g + h``."""
        return cost + self._heuristic(node, goal, algorithm)

    def _heuristic(self, node: str, goal: str, algorithm: str) -> float:
        if algorithm == "astar":
            return self.admissible_heuristic(goal, node)
        return 0.0

    def _reconstruct(
        self,
        came_from: dict[str, MapEdge],
        source: str,
        destination: str,
        best: dict[str, float],
    ) -> tuple[list[MapEdge], float]:
        edges: list[MapEdge] = []
        node = destination
        while node != source:
            if node not in came_from:
                raise PlanningError(
                    f"route reconstruction failed at {node!r}"
                )
            edge = came_from[node]
            edges.append(edge)
            node = edge.source
        edges.reverse()
        return edges, best[destination]

    # -- public planning ----------------------------------------------------

    def plan_route(
        self,
        start_node: str,
        destination_node: str,
        required_waypoints: list[str] | tuple[str, ...] = (),
        *,
        algorithm: str | None = None,
    ) -> Route:
        algorithm = algorithm or self.algorithm
        waypoints = [str(node) for node in required_waypoints]
        if not self.track.has_node(start_node):
            raise PlanningError(f"unknown start node {start_node!r}")
        if not self.track.has_node(destination_node):
            raise PlanningError(f"unknown destination node {destination_node!r}")
        for waypoint in waypoints:
            if not self.track.has_node(waypoint):
                raise PlanningError(f"unknown required waypoint {waypoint!r}")

        legs: list[tuple[str, str]] = []
        cursor = start_node
        for waypoint in [*waypoints, destination_node]:
            if waypoint == cursor:
                continue
            legs.append((cursor, waypoint))
            cursor = waypoint

        steps: list[RouteStep] = []
        nodes: list[str] = [start_node]
        total = 0.0
        for leg_source, leg_target in legs:
            edges, _cost = self.search(leg_source, leg_target, algorithm=algorithm)
            for edge in edges:
                step = RouteStep(
                    index=len(steps),
                    edge=edge,
                    travel_time_s=edge.travel_time_s(),
                    overhead_s=edge.overhead_s,
                    cost_s=edge.cost_s(),
                )
                steps.append(step)
                nodes.append(edge.destination)
                total += step.cost_s

        route_id = _route_id(algorithm, start_node, destination_node, waypoints, nodes)
        return Route(
            route_id=route_id,
            start_node=start_node,
            destination_node=destination_node,
            required_waypoints=tuple(waypoints),
            algorithm=algorithm,
            nodes=tuple(nodes),
            steps=tuple(steps),
            total_cost_s=total,
            heuristic_admissible=(
                algorithm != "astar" or self.metric_consistent()
            ),
        )


def _route_id(
    algorithm: str,
    start: str,
    destination: str,
    waypoints: list[str],
    nodes: list[str],
) -> str:
    payload = "|".join(
        [algorithm, f"{start}->{destination}", ",".join(waypoints), ",".join(nodes)]
    )
    digest = hashlib.sha1(payload.encode("utf-8")).hexdigest()[:12]
    return f"route-{digest}"


def plan_route(
    track: TrackMap,
    start_node: str,
    destination_node: str,
    required_waypoints: list[str] | tuple[str, ...] = (),
    *,
    algorithm: str = "dijkstra",
) -> Route:
    """Convenience wrapper mirroring the requested ``plan_route`` signature."""
    return Planner(track, algorithm=algorithm).plan_route(
        start_node, destination_node, required_waypoints
    )


__all__ = [
    "ALGORITHMS",
    "Planner",
    "Route",
    "RouteStep",
    "plan_route",
]
