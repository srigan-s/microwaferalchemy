"""Route execution driven by localization, not by elapsed time.

Rules enforced here:

* The start node is confirmed by the localization source before any motion.
* A FOLLOW_EDGE step ends when the localization source confirms the destination
  node, never because a timer expired or because the edge pattern flipped.
* A SWITCH_EDGE step receives a bound :class:`SwitchAuthorization` covering the
  source node, both edges, and the switch location, and needs destination
  confirmation after crossing.
* Any operator confirmation happens with the wheels stopped.
* Every step is bounded, and the wheels are stopped in ``finally``.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import Any

from ..errors import (
    AuthorizationError,
    ConfigError,
    ExecutionError,
    I2CError,
    LocalizationError,
    MapError,
    SafetyError,
)
from ..geometry import CrossingInputs, evaluate_crossing
from ..localization import (
    Localization,
    LocalizationConfirmer,
    LocalizationPolicy,
)
from ..navconfig import NavConfig
from ..safety import FaultCode, FaultRecord, SafetyState
from ..sensing.follower import EdgeFollower, FollowStopReason
from ..sensing.switching import EdgeSwitcher, SwitchAuthorization
from .graph import EdgeAction, TrackMap, TravelDirection
from .planner import Route, RouteStep


@dataclass(frozen=True)
class StepResult:
    index: int
    action: str
    source: str
    destination: str
    completed: bool
    reason: str
    duration_s: float
    detail: dict[str, Any] | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "action": self.action,
            "source": self.source,
            "destination": self.destination,
            "completed": self.completed,
            "reason": self.reason,
            "duration_s": self.duration_s,
            "detail": self.detail,
        }


@dataclass(frozen=True)
class ExecutionResult:
    completed: bool
    dry_run: bool
    reason: str
    route: Route
    steps: tuple[StepResult, ...]
    elapsed_s: float
    final_localization: Localization | None = None
    fault: FaultRecord | None = None

    def as_dict(self) -> dict[str, Any]:
        return {
            "completed": self.completed,
            "dry_run": self.dry_run,
            "reason": self.reason,
            "route": self.route.as_dict(),
            "elapsed_s": self.elapsed_s,
            "steps": [step.as_dict() for step in self.steps],
            "final_localization": (
                {
                    "node_id": self.final_localization.node_id,
                    "marker_id": self.final_localization.marker_id,
                    "source": self.final_localization.source,
                }
                if self.final_localization
                else None
            ),
            "fault": self.fault.code.value if self.fault else None,
        }


class RouteExecutor:
    """Executes a planned :class:`~waferbot.nav.planner.Route`."""

    def __init__(
        self,
        robot,
        track: TrackMap,
        *,
        nav: NavConfig | None = None,
        arrival_monitor=None,
        confirm: Callable[[str], bool] | None = None,
        clock: Callable[[], float] | None = None,
        telemetry=None,
        follower_factory: Callable[..., EdgeFollower] | None = None,
        switcher_factory: Callable[..., EdgeSwitcher] | None = None,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.robot = robot
        self.track = track.validate()
        self.nav = (nav or NavConfig()).validate()
        self.arrival_monitor = arrival_monitor
        self.confirm = confirm
        self.clock = clock or getattr(robot, "clock", time.monotonic)
        self.sleep = sleep
        self.telemetry = telemetry
        self._follower_factory = follower_factory
        self._switcher_factory = switcher_factory
        self._period_s = 1.0 / self.nav.follow.rate_hz
        self._confirmed_node: str | None = None
        self._last_localization: Localization | None = None
        self._physical = False
        self._marker_to_node = self.track.marker_to_node()
        self._shutdown_done = False
        self._localization_policy = LocalizationPolicy(
            max_age_s=self.nav.execution.localization_max_age_s,
            min_confidence=self.nav.execution.localization_min_confidence,
        )

    # -- construction helpers ----------------------------------------------

    def _make_follower(self) -> EdgeFollower:
        if self._follower_factory is not None:
            return self._follower_factory(
                robot=self.robot,
                config=self.nav.follow,
                telemetry=self.telemetry,
                clock=self.clock,
                sleep=self.sleep,
            )
        return EdgeFollower(
            self.robot,
            config=self.nav.follow,
            clock=self.clock,
            sleep=self.sleep,
            telemetry=self.telemetry,
        )

    def _make_switcher(
        self,
        *,
        speed_cap: int | None = None,
        config=None,
    ) -> EdgeSwitcher:
        provider = self._location_provider()
        config = config or self.nav.switch
        if speed_cap is not None:
            if speed_cap < 1:
                raise ConfigError(
                    "switch speed ceiling is below one PWM count; refusing to "
                    "round the calibrated limit up"
                )
            config = replace(config, speed=min(config.speed, speed_cap))
        counts_to_mps = self.nav.counts_to_mps
        if self._switcher_factory is not None:
            return self._switcher_factory(
                robot=self.robot,
                config=config,
                geometry=self.nav.geometry,
                telemetry=self.telemetry,
                clock=self.clock,
                sleep=self.sleep,
                confirm=self.confirm,
                location_provider=provider,
                counts_to_mps=counts_to_mps,
                require_distance_bound=self._physical,
                marker_to_node=self._marker_to_node,
            )
        return EdgeSwitcher(
            self.robot,
            config=config,
            geometry=self.nav.geometry,
            clock=self.clock,
            sleep=self.sleep,
            telemetry=self.telemetry,
            confirm=self.confirm,
            location_provider=provider,
            counts_to_mps=counts_to_mps,
            require_distance_bound=self._physical,
            marker_to_node=self._marker_to_node,
        )

    def _location_provider(self):
        monitor = self.arrival_monitor
        if monitor is None or monitor.requires_stop():
            # Manual confirmation is handled by the switcher's ``confirm`` hook
            # so that it can stop the wheels first.
            return None
        return lambda expected: monitor.check(expected)

    # -- public API ---------------------------------------------------------

    def execute_route(
        self,
        route: Route,
        *,
        dry_run: bool = False,
        enforce_map_readiness: bool = True,
        allow_example_map: bool | None = None,
    ) -> ExecutionResult:
        started = self.clock()
        self._physical = bool(enforce_map_readiness)
        allow_example = (
            self.nav.execution.allow_example_map
            if allow_example_map is None
            else allow_example_map
        )
        readiness_error = self._readiness_error(enforce_map_readiness, allow_example)
        step_plan_error = self._validate_route(route)

        if dry_run:
            if step_plan_error is not None:
                return ExecutionResult(
                    completed=False,
                    dry_run=True,
                    reason=step_plan_error,
                    route=route,
                    steps=(),
                    elapsed_s=self.clock() - started,
                )
            reason = "dry run: route validated without touching the robot"
            if readiness_error is not None:
                reason += f"; map note: {readiness_error}"
            return ExecutionResult(
                completed=True,
                dry_run=True,
                reason=reason,
                route=route,
                steps=tuple(
                    StepResult(
                        index=step.index,
                        action=step.action.value,
                        source=step.source,
                        destination=step.destination,
                        completed=True,
                        reason="planned",
                        duration_s=0.0,
                    )
                    for step in route.steps
                ),
                elapsed_s=self.clock() - started,
            )

        if readiness_error is not None:
            raise MapError(readiness_error)
        if step_plan_error is not None:
            raise ExecutionError(step_plan_error)
        if enforce_map_readiness:
            self._require_speed_calibration(route)
        if not self.robot.armed:
            raise SafetyError("route execution requires an armed robot")
        if self.arrival_monitor is None and self.confirm is None:
            raise LocalizationError(
                "route execution needs a localization source: pass a marker "
                "localizer or a manual confirmation callback"
            )

        results: list[StepResult] = []
        final: Localization | None = None
        completed = False
        overall_deadline = started + self.nav.execution.max_route_duration_s

        try:
            start_localization = self._confirm_node(
                route.start_node,
                self.clock() + self.nav.switch.establish_timeout_s,
                purpose=f"route start, facing {self.track.node(route.start_node).heading:g} deg in map frame",
            )
            if start_localization is None:
                reason = (
                    "could not confirm the robot is at start node "
                    f"{route.start_node}"
                )
                self._fail(FaultCode.LOCALIZATION_FAILURE, reason)
                return self._result(False, reason, route, results, started, None)
            self._confirmed_node = route.start_node
            final = start_localization
            self.robot.set_state(SafetyState.FOLLOWING)

            for step in route.steps:
                if self.clock() >= overall_deadline:
                    reason = "route exceeded the maximum allowed duration"
                    self._fail(FaultCode.SWITCH_TIMEOUT, reason)
                    return self._result(False, reason, route, results, started, final)
                if self.robot.fault is not None:
                    reason = f"robot fault {self.robot.fault.code.value}"
                    return self._result(False, reason, route, results, started, final)

                step_started = self.clock()
                step_result = self._execute_step(step, route, overall_deadline)
                step_result = replace(
                    step_result, duration_s=self.clock() - step_started
                )
                results.append(step_result)
                if not step_result.completed:
                    return self._result(
                        False, step_result.reason, route, results, started, final
                    )
                self._confirmed_node = step.destination
                final = self._last_localization or final

            completed = True
            reason = (
                f"arrived at {route.destination_node} after "
                f"{len(route.steps)} step(s)"
            )
            if not self._shutdown():
                completed = False
                reason = f"{reason}; mandatory final stop failed"
            self._shutdown_done = completed or self.robot.fault is not None
            if self.robot.fault is None:
                self.robot.set_state(
                    SafetyState.COMPLETE if completed else SafetyState.IDLE
                )
            return self._result(completed, reason, route, results, started, final)
        except (I2CError, SafetyError, AuthorizationError, LocalizationError) as exc:
            reason = f"route aborted: {exc}"
            if self.robot.fault is None:
                self._fail(FaultCode.MOTOR_COMMUNICATION_FAILURE, reason)
            return self._result(False, reason, route, results, started, final)
        finally:
            if not self._shutdown_done:
                # Reached only when an unexpected path skipped the explicit
                # shutdown; the success result was already built before this.
                try:
                    self.robot.stop()
                except I2CError:
                    completed = False
            if not completed and self.robot.fault is None:
                self.robot.set_state(SafetyState.IDLE)

    # -- steps --------------------------------------------------------------

    def _execute_step(
        self, step: RouteStep, route: Route, overall_deadline: float
    ) -> StepResult:
        edge = step.edge
        if edge.action is EdgeAction.FOLLOW_EDGE:
            return self._follow_edge(step, overall_deadline)
        if edge.action is EdgeAction.SWITCH_EDGE:
            return self._switch_edge(step, route, overall_deadline)
        if edge.action is EdgeAction.TURN:
            return self._turn(step, overall_deadline)
        if edge.action is EdgeAction.DOCK:
            return self._dock(step, overall_deadline)
        if edge.action is EdgeAction.STOP:
            if edge.source != edge.destination:
                # Non-coincident STOP actions are rejected by map validation;
                # keep a positive localization requirement as a defensive check.
                return self._confirm_step(step, "stop")
            self.robot.stop()
            return StepResult(
                index=step.index,
                action=edge.action.value,
                source=step.source,
                destination=step.destination,
                completed=True,
                reason="stop action executed",
                duration_s=0.0,
            )
        raise ExecutionError(f"unsupported action {edge.action!r}")

    def _follow_edge(self, step: RouteStep, overall_deadline: float) -> StepResult:
        edge = step.edge
        if (edge.direction or TravelDirection.FORWARD) is TravelDirection.REVERSE:
            raise ExecutionError(
                f"edge {edge.source}->{edge.destination} is marked "
                "direction=reverse, and reverse edge following is not supported; "
                "use a forward edge (DOCK supports reverse)"
            )
        target_edge = self.track.edge_state_for(
            edge.edge_side, where=f"edge {edge.source}->{edge.destination}"
        )
        follower = self._make_follower()
        follower.marker_to_node = self._marker_to_node
        budget = min(
            self.nav.execution.max_edge_travel_s,
            max(0.001, overall_deadline - self.clock()),
        )
        self.robot.set_state(SafetyState.FOLLOWING)
        result = follower.follow_edge(
            target_edge,
            max_duration_s=budget,
            arrival_monitor=self.arrival_monitor,
            expected_node=edge.destination,
            max_speed_pwm=self._speed_cap_pwm(edge),
        )
        detail = result.as_dict()
        if result.stop_reason is FollowStopReason.ARRIVAL:
            # Adopt only the localization the follower actually accepted; never
            # synthesise arrival evidence from the planned destination.
            self._adopt_localization(
                result.arrival_localization, edge.destination, purpose="arrival"
            )
            return StepResult(
                index=step.index,
                action=edge.action.value,
                source=step.source,
                destination=step.destination,
                completed=True,
                reason=f"arrived at {edge.destination}",
                duration_s=result.elapsed_s,
                detail=detail,
            )
        reason = (
            f"follow step ended with {result.stop_reason.value} before arrival "
            f"at {edge.destination}"
        )
        if self.robot.fault is None:
            self._fail(FaultCode.LOCALIZATION_FAILURE, reason)
        return StepResult(
            index=step.index,
            action=edge.action.value,
            source=step.source,
            destination=step.destination,
            completed=False,
            reason=reason,
            duration_s=result.elapsed_s,
            detail=detail,
        )

    def _switch_edge(
        self, step: RouteStep, route: Route, overall_deadline: float
    ) -> StepResult:
        edge = step.edge
        where = f"switch {edge.source}->{edge.destination}"
        current_edge = self.track.edge_state_for(edge.edge_side, where=where)
        target_side = self.track.edge_target_side(edge)
        target_edge = self.track.edge_state_for(target_side, where=where)

        location_id = self._confirmed_node
        if location_id != edge.source:
            confirmed = self._confirm_node(
                edge.source,
                self.clock() + self.nav.switch.establish_timeout_s,
                purpose="switch source",
            )
            if confirmed is None:
                reason = f"could not confirm the switch source node {edge.source}"
                self._fail(FaultCode.LOCALIZATION_FAILURE, reason)
                return StepResult(
                    index=step.index,
                    action=edge.action.value,
                    source=step.source,
                    destination=step.destination,
                    completed=False,
                    reason=reason,
                    duration_s=0.0,
                )
            location_id = edge.source

        authorization = SwitchAuthorization(
            authorized=True,
            route_id=route.route_id,
            source_node=edge.source,
            source_edge=current_edge,
            target_edge=target_edge,
            location_id=edge.source,
            destination_location_id=edge.destination,
            issued_at=self.clock(),
        )
        self.robot.set_state(SafetyState.SWITCHING)
        # Each switch step runs with the geometry declared on its own edge:
        # an explicit angle means a diagonal crossing, otherwise lateral.
        angle = edge.crossing_angle_deg
        if angle is None or float(angle) >= 89.999:
            mode, angle_value = "lateral", None
        else:
            mode, angle_value = "diagonal", float(angle)
        switch_config = replace(
            self.nav.switch, mode=mode, crossing_angle_deg=angle_value
        )
        switcher = self._make_switcher(
            speed_cap=self._speed_cap_pwm(edge), config=switch_config
        )
        result = switcher.switch_edge(
            current_edge,
            target_edge,
            authorization=authorization,
            location_id=location_id,
            destination_monitor=self.arrival_monitor,
            deadline=overall_deadline,
        )
        detail = result.as_dict()
        if result.completed:
            self._adopt_localization(
                result.destination, edge.destination, purpose="switch destination"
            )
            return StepResult(
                index=step.index,
                action=edge.action.value,
                source=step.source,
                destination=step.destination,
                completed=True,
                reason=result.reason,
                duration_s=result.elapsed_s,
                detail=detail,
            )
        return StepResult(
            index=step.index,
            action=edge.action.value,
            source=step.source,
            destination=step.destination,
            completed=False,
            reason=result.reason,
            duration_s=result.elapsed_s,
            detail=detail,
        )

    def _turn(self, step: RouteStep, overall_deadline: float) -> StepResult:
        edge = step.edge
        speed = self._capped_speed(
            self.nav.execution.turn_speed, self._speed_cap_pwm(edge), "TURN"
        )
        delta = self._heading_delta(edge)
        if abs(delta) >= 1e-9:
            # Heading is an absolute chassis orientation, so the rotation sense
            # comes from the map delta alone: positive is counter-clockwise
            # (left). `direction` does not flip it (and reverse TURN edges are
            # rejected by map validation).
            turn_left = delta > 0
            rotate = self.robot.rotate_left if turn_left else self.robot.rotate_right
            self.robot.run_command(
                lambda: rotate(speed),
                edge.turn_time_s,
                deadline=overall_deadline,
                context="turn",
            )
        else:
            self.robot.stop()
        return self._confirm_step(step, "turn")

    def _dock(self, step: RouteStep, overall_deadline: float) -> StepResult:
        edge = step.edge
        speed = self._capped_speed(
            self.nav.execution.dock_speed, self._speed_cap_pwm(edge), "DOCK"
        )
        if (edge.direction or TravelDirection.FORWARD) is TravelDirection.REVERSE:
            move = self.robot.backward
        else:
            move = self.robot.forward
        self.robot.run_command(
            lambda: move(speed),
            edge.dock_time_s,
            deadline=overall_deadline,
            context="dock",
        )
        return self._confirm_step(step, "dock")

    def _confirm_step(self, step: RouteStep, kind: str) -> StepResult:
        edge = step.edge
        purpose = f"{kind} arrival"
        if edge.action is EdgeAction.TURN:
            # A timed rotation is not self-verifying: ask explicitly about the
            # absolute heading convention as well as the node identity.
            heading = float(self.track.node(edge.destination).heading)
            purpose = (
                f"{kind} arrival: the robot should now be at "
                f"{edge.destination} facing {heading:.0f} deg in map frame"
            )
        found = self._confirm_node(
            edge.destination,
            self.clock() + self.nav.switch.verify_timeout_s,
            purpose=purpose,
        )
        if found is None:
            reason = (
                f"{kind} finished but {edge.destination} was not confirmed; "
                "elapsed time is not arrival evidence"
            )
            self._fail(FaultCode.LOCALIZATION_FAILURE, reason)
            return StepResult(
                index=step.index,
                action=edge.action.value,
                source=step.source,
                destination=step.destination,
                completed=False,
                reason=reason,
                duration_s=0.0,
            )
        return StepResult(
            index=step.index,
            action=edge.action.value,
            source=step.source,
            destination=step.destination,
            completed=True,
            reason=f"{kind} complete and confirmed at {edge.destination}",
            duration_s=0.0,
        )

    # -- helpers ------------------------------------------------------------

    def _speed_cap_pwm(self, edge) -> int | None:
        """Convert a graph speed ceiling into a PWM cap, if calibrated."""
        if not self.nav.has_speed_calibration:
            return None
        limit_mps = float(edge.speed_limit_mps)
        if limit_mps <= 0:
            return None
        counts = limit_mps / float(self.nav.counts_to_mps)
        if counts < 1.0:
            raise ConfigError(
                f"edge {edge.source}->{edge.destination} speed limit "
                f"{limit_mps} m/s is below one PWM count ({counts:.3f}) with the "
                "current counts_to_mps factor; refusing to round the limit up"
            )
        return int(min(self.robot.config.motor.max_speed, counts))

    def _capped_speed(self, base: int, cap: int | None, label: str) -> int:
        """Apply a calibrated ceiling to a timed manoeuvre's speed."""
        if cap is None:
            return min(base, self.robot.config.motor.max_speed)
        if cap < 1:
            raise ConfigError(
                f"{label} speed ceiling is below one PWM count; refusing to "
                "round the calibrated limit up"
            )
        return max(1, min(base, cap))

    def _heading_delta(self, edge) -> float:
        """Signed heading change for a TURN edge, in degrees.

        Convention: headings are degrees in the map frame, 0 = +x, increasing
        counter-clockwise. A positive delta means a left (counter-clockwise)
        rotation of the robot. An exactly 180-degree change is geometrically
        ambiguous; it resolves to -180 (clockwise) by convention.
        """
        start = float(self.track.node(edge.source).heading)
        end = float(self.track.node(edge.destination).heading)
        return (end - start + 180.0) % 360.0 - 180.0

    def _require_speed_calibration(self, route: Route) -> None:
        """Physical graph execution must honour the map's m/s ceilings.

        Without a measured counts-to-m/s factor the map's speed limits cannot be
        enforced on the motors, so physical execution refuses instead of driving
        at an unverified speed.
        """
        if self.nav.has_speed_calibration:
            return
        if not route.steps:
            return
        raise ConfigError(
            "physical route execution needs a measured speed calibration so the "
            "map's speed_limit_mps can be enforced; run "
            "`waferbot calibrate speed` and set nav.counts_to_mps (mock and "
            "--dry-run do not need it)"
        )

    def _confirm_node(
        self, expected: str, deadline: float, *, purpose: str
    ) -> Localization | None:
        monitor = self.arrival_monitor
        confirmer = LocalizationConfirmer(
            expected,
            policy=self._localization_policy,
            required=self.nav.execution.arrival_confirmations,
            clock=self.clock,
            marker_to_node=self._marker_to_node,
        )
        if monitor is None:
            if self.confirm is None:
                raise LocalizationError(
                    f"{purpose}: no localization source is configured for {expected}"
                )
            self.robot.stop()
            if self.confirm(
                f"{purpose}: is the robot at {expected}? (motors are stopped)"
            ):
                found = Localization(
                    node_id=expected,
                    marker_id=expected,
                    confidence=1.0,
                    timestamp=self.clock(),
                    source="manual",
                )
                confirmer.offer(found)
                if confirmer.complete:
                    self._record_localization(confirmer.latest, expected)
                    return confirmer.latest
            return None
        while self.clock() < deadline:
            if monitor.requires_stop():
                self.robot.stop()
            contextual_check = getattr(monitor, "check_with_context", None)
            evidence = (contextual_check(expected, purpose) if contextual_check
                        else monitor.check(expected))
            confirmer.offer(evidence)
            if confirmer.complete:
                self._record_localization(confirmer.latest, expected)
                return confirmer.latest
            self.sleep(self._period_s)
        return None

    def _record_localization(self, localization: Localization, expected: str) -> None:
        self._last_localization = localization
        if self.telemetry is not None:
            self.telemetry.localization(localization, expected=expected)

    def _adopt_localization(
        self, localization: Localization | None, expected: str, *, purpose: str
    ) -> None:
        """Record real localization evidence; never fabricate one."""
        if localization is None:
            raise LocalizationError(
                f"{purpose}: no localization evidence was produced for {expected}"
            )
        self._record_localization(localization, expected)

    def _readiness_error(self, enforce: bool, allow_example: bool) -> str | None:
        if not enforce:
            return None
        try:
            self.track.require_physical_ready(allow_example=allow_example)
        except MapError as exc:
            return str(exc)
        return None

    def _validate_route(self, route: Route) -> str | None:
        """Validate the actual route before any motion.

        Checks start/destination membership, node-sequence continuity, step
        continuity, membership in the *current* map (so forged or stale routes
        are rejected), enabled status, direction support, per-step edge sides,
        declared geometry, and ordered mandatory waypoints.
        """
        if not route.steps and route.nodes != (route.start_node,):
            return (
                f"route for {route.start_node} has no steps but its node sequence "
                f"is {route.nodes!r}"
            )
        for node_id in (route.start_node, route.destination_node):
            if not self.track.has_node(node_id):
                return f"route node {node_id!r} is not in the current map"
        if not self._contains_in_order(route.nodes, route.required_waypoints):
            return (
                f"route node sequence {route.nodes!r} does not visit required "
                f"waypoints {route.required_waypoints!r} in order"
            )
        if len(route.nodes) != len(route.steps) + 1:
            return (
                f"route declares {len(route.nodes)} nodes for "
                f"{len(route.steps)} steps"
            )
        if route.nodes[0] != route.start_node or route.nodes[-1] != route.destination_node:
            return (
                f"route node sequence {route.nodes!r} does not run from "
                f"{route.start_node!r} to {route.destination_node!r}"
            )

        cursor = route.start_node
        for index, step in enumerate(route.steps):
            edge = step.edge
            where = f"{edge.source}->{edge.destination}"
            if step.index != index:
                return f"step {index} carries index {step.index}"
            if edge.source != cursor:
                return (
                    f"step {index} ({where}) does not continue from {cursor!r}"
                )
            if route.nodes[index + 1] != edge.destination:
                return (
                    f"step {index} ({where}) does not match node sequence entry "
                    f"{route.nodes[index + 1]!r}"
                )
            canonical = self.track.edge_matching(edge)
            if canonical is None:
                return f"step {index} ({where}) is not an edge in the current map"
            if not canonical.enabled:
                return f"step {index} ({where}) is disabled"
            if (
                canonical.distance_m != edge.distance_m
                or canonical.speed_limit_mps != edge.speed_limit_mps
                or canonical.crossing_angle_deg != edge.crossing_angle_deg
            ):
                return (
                    f"step {index} ({where}) disagrees with the map's own values "
                    "(stale or forged route)"
                )
            if edge.action is EdgeAction.STOP and edge.source != edge.destination:
                return (
                    f"step {index} ({where}) is a spatial STOP; STOP must be "
                    "stationary and self/coincident"
                )
            if (
                edge.action is EdgeAction.FOLLOW_EDGE
                and (edge.direction or TravelDirection.FORWARD)
                is TravelDirection.REVERSE
            ):
                return (
                    f"step {index} ({where}) is a reverse FOLLOW edge, which is "
                    "not supported"
                )
            try:
                if edge.action in (EdgeAction.FOLLOW_EDGE, EdgeAction.SWITCH_EDGE):
                    self.track.edge_state_for(
                        edge.edge_side, where=where
                    )
                if edge.action is EdgeAction.SWITCH_EDGE:
                    target = self.track.edge_target_side(edge)
                    self.track.edge_state_for(
                        target, where=f"{where} target"
                    )
                    angle = edge.crossing_angle_deg
                    if angle is not None and float(angle) < 89.999:
                        # Per-edge diagonal geometry must be proven before any
                        # motion, not discovered mid-crossing.
                        self.nav.geometry.require_measurements(
                            f"diagonal switch {where}"
                        )
                        evaluation = evaluate_crossing(
                            CrossingInputs.from_config(self.nav.geometry),
                            float(angle),
                        )
                        if not evaluation.unambiguous_geometry:
                            return (
                                f"diagonal switch {where} is not observable with "
                                "the measured geometry: "
                                + "; ".join(evaluation.warnings[1:])
                            )
            except (MapError, ConfigError) as exc:
                return str(exc)
            cursor = edge.destination
        return None

    @staticmethod
    def _contains_in_order(
        nodes: tuple[str, ...], waypoints: tuple[str, ...]
    ) -> bool:
        position = 0
        for waypoint in waypoints:
            while position < len(nodes) and nodes[position] != waypoint:
                position += 1
            if position >= len(nodes):
                return False
            position += 1
        return True

    def _fail(self, code: FaultCode, reason: str) -> None:
        existing = self.robot.fault
        if existing is not None:
            # Keep the first failure; append context instead of relabelling it.
            self.robot.safety.note_failure(f"{code.value}: {reason}")
            return
        try:
            self.robot.raise_fault(code, reason)
        except I2CError:
            pass
        if self.telemetry is not None and self.robot.fault is not None:
            self.telemetry.fault(self.robot.fault)

    def _shutdown(self) -> bool:
        """Mandatory stop before reporting success. False when it failed."""
        try:
            self.robot.stop()
        except I2CError:
            return False
        return True

    def _result(
        self,
        completed: bool,
        reason: str,
        route: Route,
        steps: list[StepResult],
        started: float,
        final: Localization | None,
    ) -> ExecutionResult:
        return ExecutionResult(
            completed=completed,
            dry_run=False,
            reason=reason,
            route=route,
            steps=tuple(steps),
            elapsed_s=self.clock() - started,
            final_localization=final,
            fault=self.robot.fault,
        )


__all__ = ["ExecutionResult", "RouteExecutor", "StepResult"]
