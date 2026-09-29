"""Edge-switching finite state machine.

Required phases::

    FOLLOW_EDGE -> APPROACH_SWITCH_LOCATION -> CONFIRM_SWITCH_LOCATION ->
    REDUCE_SPEED -> EXECUTE_SWITCH -> SEARCH_FOR_OPPOSITE_EDGE ->
    VERIFY_OPPOSITE_EDGE -> RESUME_FOLLOWING

Safety rules that make this more than a strafe timer:

* The route executor must supply a :class:`SwitchAuthorization` binding the
  expected source node, the current edge, the target edge, and the switch
  location. A mismatched authorization refuses to move the robot.
* The current edge has to be *established* from fresh sensor readings before any
  crossing starts.
* The switch location has to be confirmed by an independent localization source
  (marker reader) or an explicit operator confirmation with the wheels stopped.
* The opposite edge only counts after several consecutive, monotonically fresh,
  debounced readings, and (by default) after the destination location is
  independently confirmed. Seeing the other edge is not proof of arrival.
* Every phase is bounded by time, and the wheels are stopped in ``finally``.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum

from ..errors import AuthorizationError, ConfigError, I2CError, SafetyError, SensorError
from ..geometry import CrossingInputs, evaluate_crossing
from ..localization import Localization, LocalizationConfirmer, LocalizationPolicy
from ..navconfig import GeometryConfig, SwitchConfig
from ..safety import FaultCode, FaultRecord, SafetyState
from .edge import EdgeDetection, EdgeDetector, EdgeState


class SwitchPhase(str, Enum):
    FOLLOW_EDGE = "FOLLOW_EDGE"
    APPROACH_SWITCH_LOCATION = "APPROACH_SWITCH_LOCATION"
    CONFIRM_SWITCH_LOCATION = "CONFIRM_SWITCH_LOCATION"
    REDUCE_SPEED = "REDUCE_SPEED"
    EXECUTE_SWITCH = "EXECUTE_SWITCH"
    SEARCH_FOR_OPPOSITE_EDGE = "SEARCH_FOR_OPPOSITE_EDGE"
    VERIFY_OPPOSITE_EDGE = "VERIFY_OPPOSITE_EDGE"
    RESUME_FOLLOWING = "RESUME_FOLLOWING"
    COMPLETE = "COMPLETE"
    FAULT = "FAULT"


@dataclass(frozen=True)
class SwitchAuthorization:
    """Permission from the route executor to switch at one known place."""

    authorized: bool
    route_id: str
    source_node: str
    source_edge: EdgeState | str
    target_edge: EdgeState | str
    location_id: str
    destination_location_id: str | None = None
    issued_at: float | None = None

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "source_edge",
            self.source_edge
            if isinstance(self.source_edge, EdgeState)
            else EdgeState.from_name(self.source_edge),
        )
        object.__setattr__(
            self,
            "target_edge",
            self.target_edge
            if isinstance(self.target_edge, EdgeState)
            else EdgeState.from_name(self.target_edge),
        )

    def validate(
        self,
        *,
        current_edge: EdgeState,
        target_edge: EdgeState,
        location_id: str | None,
        now: float,
        max_age_s: float,
    ) -> None:
        """Raise :class:`AuthorizationError` unless every binding matches."""
        if not self.authorized:
            raise AuthorizationError(
                "edge switch refused: the route executor did not authorize it"
            )
        if self.source_edge is not current_edge:
            raise AuthorizationError(
                "edge switch refused: authorization is for "
                f"{self.source_edge.value} but the robot follows "
                f"{current_edge.value}"
            )
        if self.target_edge is not target_edge:
            raise AuthorizationError(
                "edge switch refused: authorization targets "
                f"{self.target_edge.value} but {target_edge.value} was requested"
            )
        if target_edge is current_edge:
            raise AuthorizationError(
                "edge switch refused: "
                f"{target_edge.value} is already the current edge"
            )
        if not self.source_node or self.source_node != self.location_id:
            raise AuthorizationError(
                "edge switch refused: authorization must name the same source "
                f"node and location (got {self.source_node!r} and "
                f"{self.location_id!r})"
            )
        if location_id is None:
            raise AuthorizationError(
                "edge switch refused: the robot has no confirmed switch "
                f"location; expected {self.location_id!r}"
            )
        if location_id != self.location_id:
            raise AuthorizationError(
                "edge switch refused: robot is at "
                f"{location_id!r}, authorization is for {self.location_id!r}"
            )
        if self.issued_at is not None:
            age = now - self.issued_at
            if age < -max_age_s or age > max_age_s:
                raise AuthorizationError(
                    "edge switch refused: authorization is stale "
                    f"(issued {age:.1f}s ago, limit {max_age_s:.1f}s)"
                )


@dataclass(frozen=True)
class SwitchResult:
    completed: bool
    reason: str
    current_edge: EdgeState
    target_edge: EdgeState
    phases: tuple[tuple[str, float], ...]
    elapsed_s: float
    travelled_s: float
    verification_samples: int
    destination: Localization | None = None
    fault: FaultRecord | None = None
    geometry: dict[str, object] | None = None

    def as_dict(self) -> dict[str, object]:
        return {
            "completed": self.completed,
            "reason": self.reason,
            "current_edge": self.current_edge.value,
            "target_edge": self.target_edge.value,
            "phases": [{"phase": phase, "t": stamp} for phase, stamp in self.phases],
            "elapsed_s": self.elapsed_s,
            "travelled_s": self.travelled_s,
            "verification_samples": self.verification_samples,
            "destination": (
                {
                    "node_id": self.destination.node_id,
                    "source": self.destination.source,
                }
                if self.destination
                else None
            ),
            "fault": self.fault.code.value if self.fault else None,
            "geometry": self.geometry,
        }


@dataclass
class _Trace:
    phases: list[tuple[str, float]] = field(default_factory=list)

    def mark(self, phase: SwitchPhase, clock: Callable[[], float]) -> None:
        self.phases.append((phase.value, clock()))


class EdgeSwitcher:
    """Runs the bound switching FSM for one manoeuvre."""

    def __init__(
        self,
        robot,
        *,
        detector: EdgeDetector | None = None,
        config: SwitchConfig | None = None,
        geometry: GeometryConfig | None = None,
        clock: Callable[[], float] | None = None,
        sleep: Callable[[float], None] = time.sleep,
        telemetry=None,
        confirm: Callable[[str], bool] | None = None,
        location_provider: Callable[[str], Localization | None] | None = None,
        counts_to_mps: float | None = None,
        require_distance_bound: bool = False,
        marker_to_node: dict[str, str] | None = None,
    ) -> None:
        self.robot = robot
        self.config = (config or SwitchConfig()).validate()
        self.geometry = (geometry or GeometryConfig()).validate()
        self.clock = clock or getattr(robot, "clock", time.monotonic)
        self.sleep = sleep
        self.telemetry = telemetry
        self.confirm = confirm
        self.location_provider = location_provider
        self.counts_to_mps = counts_to_mps
        self.require_distance_bound = bool(require_distance_bound)
        self.marker_to_node = marker_to_node
        self.detector = detector or EdgeDetector(
            stable_samples=self.config.stable_samples,
            max_age_s=self.config.sensor_max_age_s,
            clock=self.clock,
        )
        self._localization_policy = LocalizationPolicy(
            max_age_s=self.config.localization_max_age_s,
            min_confidence=self.config.localization_min_confidence,
        )
        self._period_s = 1.0 / float(self.config.poll_rate_hz)

    # -- public API ---------------------------------------------------------

    def switch_edge(
        self,
        current_edge: EdgeState | str,
        target_edge: EdgeState | str,
        *,
        authorization: SwitchAuthorization,
        location_id: str | None = None,
        destination_monitor=None,
        deadline: float | None = None,
    ) -> SwitchResult:
        """Execute the full FSM.

        Authorization problems raise before anything moves; bounded manoeuvre
        failures latch a fault and return ``completed=False``.
        """
        current = (
            current_edge
            if isinstance(current_edge, EdgeState)
            else EdgeState.from_name(current_edge)
        )
        target = (
            target_edge
            if isinstance(target_edge, EdgeState)
            else EdgeState.from_name(target_edge)
        )
        if current not in (EdgeState.BLACK_LEFT, EdgeState.BLACK_RIGHT):
            raise ValueError(f"{current.value} is not a followable edge")
        if target not in (EdgeState.BLACK_LEFT, EdgeState.BLACK_RIGHT):
            raise ValueError(f"{target.value} is not a followable edge")
        if not self.robot.armed:
            raise SafetyError("edge switching requires an armed robot")

        now = self.clock()
        authorization.validate(
            current_edge=current,
            target_edge=target,
            location_id=location_id,
            now=now,
            max_age_s=self.config.authorization_max_age_s,
        )
        if (
            self.config.require_destination_confirmation
            and destination_monitor is None
            and self.location_provider is None
            and self.confirm is None
        ):
            raise AuthorizationError(
                "edge switch refused: destination confirmation is required but "
                "no localization source or operator confirmation was provided"
            )
        if self.require_distance_bound:
            # Fail fast on a misconfigured physical switch, before any motion.
            if self.counts_to_mps is None:
                raise ConfigError(
                    "physical switching requires a measured speed calibration "
                    "(nav.counts_to_mps) so switch.max_travel_m can be enforced"
                )
            if self.config.max_travel_m <= 0:
                raise ConfigError(
                    "physical switching requires switch.max_travel_m > 0 (a "
                    "measured, conservative straight-line travel budget)"
                )

        geometry_report = self._evaluate_geometry()
        overall_deadline = deadline
        self._overall_deadline = deadline if deadline is not None else math.inf
        # Moving time (never stopped prompt time, never stopped phase time) is
        # accumulated against this budget; stopped establishment and manual
        # confirmations are therefore free.
        self._moving_s = 0.0
        self._budget_limit_s = self._budget_limit(switch_speed_candidate=self.config.speed)
        trace = _Trace()
        start = now
        self._shutdown_done = False
        travelled = 0.0
        verification_samples = 0
        destination: Localization | None = None
        reason = ""
        completed = False
        self.robot.set_state(SafetyState.SWITCHING)

        def finish(
            done: bool,
            why: str,
            *,
            verification: int = verification_samples,
            destination_hit: Localization | None = destination,
            travel: float = travelled,
        ) -> SwitchResult:
            """Stop before reporting success, so cleanup failures are visible."""
            shutdown_ok = self._shutdown()
            self._shutdown_done = shutdown_ok
            if not shutdown_ok:
                done = False
                why = f"{why}; mandatory final stop failed"
            return self._result(
                done,
                why,
                current,
                target,
                trace,
                start,
                travel,
                verification,
                destination_hit,
                geometry_report,
            )

        try:
            self.robot.stop()
            self.detector.reset()
            trace.mark(SwitchPhase.FOLLOW_EDGE, self.clock)
            detection = self._establish_current_edge(current)
            if detection is None:
                reason = f"could not establish the current edge {current.value}"
                self._fail(FaultCode.LOCALIZATION_FAILURE, reason)
                return finish(False, reason)

            trace.mark(SwitchPhase.APPROACH_SWITCH_LOCATION, self.clock)
            trace.mark(SwitchPhase.CONFIRM_SWITCH_LOCATION, self.clock)
            confirmed = self._confirm_location(
                authorization.location_id,
                self.clock() + self.config.establish_timeout_s,
            )
            if confirmed is None:
                reason = (
                    f"switch location {authorization.location_id} could not be "
                    "confirmed"
                )
                self._fail(FaultCode.LOCALIZATION_FAILURE, reason)
                return finish(False, reason)

            trace.mark(SwitchPhase.REDUCE_SPEED, self.clock)
            self.robot.stop()
            switch_speed = max(
                1, int(self.config.speed * self.config.reduce_speed_factor)
            )
            # A manual confirmation pause happens here, before the motion budget
            # starts. Re-establish the followed edge under fresh sensor evidence
            # before issuing any motion, because the operator may have moved the
            # robot (or the tape) while it was stopped.
            recheck = self._establish_current_edge(current)
            if recheck is None:
                reason = (
                    f"lost the {current.value} boundary while the robot was "
                    "stopped for confirmation"
                )
                self._fail(FaultCode.LOCALIZATION_FAILURE, reason)
                return finish(False, reason)

            trace.mark(SwitchPhase.EXECUTE_SWITCH, self.clock)
            found, spent = self._run_maneuver(
                current, target, switch_speed,
                min(self.clock() + self.config.switch_timeout_s, overall_deadline or math.inf),
            )
            travelled += spent
            self._moving_s += spent

            if not found and not self._budget_exhausted():
                trace.mark(SwitchPhase.SEARCH_FOR_OPPOSITE_EDGE, self.clock)
                found, spent = self._run_maneuver(
                    current, target, switch_speed,
                    min(self.clock() + self.config.search_timeout_s, overall_deadline or math.inf),
                )
                travelled += spent
                self._moving_s += spent
            if not found:
                reason = (
                    f"the {target.value} edge was not found within "
                    f"{self.config.switch_timeout_s + self.config.search_timeout_s:.1f}s "
                    "of allowed moving time"
                )
                self._fail(FaultCode.SWITCH_TIMEOUT, reason)
                return finish(False, reason, travel=travelled)

            trace.mark(SwitchPhase.VERIFY_OPPOSITE_EDGE, self.clock)
            verified, verification_samples = self._verify_edge(
                target, self.clock() + self.config.verify_timeout_s
            )
            if not verified:
                reason = (
                    f"{target.value} did not hold for "
                    f"{self.config.confirm_samples} consecutive fresh readings"
                )
                self._fail(FaultCode.SWITCH_TIMEOUT, reason)
                return finish(False, reason, verification=verification_samples, travel=travelled)

            if self.config.require_destination_confirmation:
                destination = self._confirm_destination(
                    authorization.destination_location_id,
                    destination_monitor,
                    self.clock() + self.config.verify_timeout_s,
                )
                if destination is None:
                    reason = (
                        "destination localization could not be confirmed after "
                        "crossing; the opposite edge alone is not proof of arrival"
                    )
                    self._fail(FaultCode.LOCALIZATION_FAILURE, reason)
                    return finish(
                        False,
                        reason,
                        verification=verification_samples,
                        destination_hit=destination,
                        travel=travelled,
                    )

            trace.mark(SwitchPhase.RESUME_FOLLOWING, self.clock)
            # The operator may have moved the chassis during destination
            # confirmation. Require fresh evidence while stopped before resume.
            self.detector.reset()
            ready, _ = self._verify_edge(
                target, min(self.clock() + self.config.verify_timeout_s, self._overall_deadline)
            )
            if not ready:
                reason = "target edge no longer verified after destination confirmation"
                self._fail(FaultCode.LOCALIZATION_FAILURE, reason)
                return finish(False, reason, verification=verification_samples, travel=travelled)
            resumed, spent = self._resume(
                target,
                switch_speed,
                min(self.clock() + self.config.max_travel_s, overall_deadline or math.inf),
            )
            travelled += spent
            self._moving_s += spent
            if not resumed:
                reason = f"lost the {target.value} edge while resuming"
                self._fail(FaultCode.SWITCH_TIMEOUT, reason)
                return finish(
                    False,
                    reason,
                    verification=verification_samples,
                    destination_hit=destination,
                    travel=travelled,
                )

            completed = True
            reason = f"switched from {current.value} to {target.value}"
            trace.mark(SwitchPhase.COMPLETE, self.clock)
            return finish(
                True,
                reason,
                verification=verification_samples,
                destination_hit=destination,
                travel=travelled,
            )
        except (I2CError, SafetyError) as exc:
            reason = f"switch aborted: {exc}"
            if self.robot.fault is None:
                self._fail(FaultCode.SWITCH_TIMEOUT, reason)
            trace.mark(SwitchPhase.FAULT, self.clock)
            return finish(
                False,
                reason,
                verification=verification_samples,
                destination_hit=destination,
                travel=travelled,
            )
        finally:
            if not getattr(self, "_shutdown_done", False):
                # Only reached when an unexpected exception escaped before the
                # result was built; the normal path already stopped and will not
                # be invalidated by a second stop attempt.
                try:
                    self.robot.stop()
                except I2CError as exc:
                    self.robot.safety.note_failure(f"switch cleanup stop failed: {exc}")
            if self.robot.fault is None and self.robot.state is SafetyState.SWITCHING:
                self.robot.set_state(SafetyState.IDLE)

    # -- phases -------------------------------------------------------------

    def _establish_current_edge(self, current: EdgeState) -> EdgeDetection | None:
        self.detector.reset()
        deadline = min(self.clock() + self.config.establish_timeout_s,
                       getattr(self, "_overall_deadline", math.inf))
        while self.clock() < deadline:
            try:
                reading = self.robot.read_line_sensors()
            except (I2CError, SensorError):
                return None
            detection = self.detector.update(reading, now=self.clock())
            if detection.stable and detection.stable_edge is current:
                return detection
            self.sleep(max(0.0, min(self._period_s, deadline - self.clock())))
        return None

    def _confirm_location(self, expected: str, deadline: float) -> Localization | None:
        if self.location_provider is not None:
            confirmer = LocalizationConfirmer(
                expected,
                policy=self._localization_policy,
                required=self.config.confirm_samples,
                clock=self.clock,
                marker_to_node=self.marker_to_node,
            )
            while self.clock() < deadline:
                confirmer.offer(self.location_provider(expected))
                if confirmer.complete:
                    return confirmer.latest
                self.sleep(self._period_s)
            return None
        if self.confirm is not None:
            # Operator confirmation always happens with the wheels stopped.
            self.robot.stop()
            if self.confirm(
                f"Confirm the robot is at switch location {expected} "
                "(motors are stopped)."
            ):
                return Localization(
                    node_id=expected,
                    marker_id=expected,
                    confidence=1.0,
                    timestamp=self.clock(),
                    source="manual",
                )
        return None

    def _confirm_destination(
        self, expected: str | None, monitor, deadline: float
    ) -> Localization | None:
        if expected is None:
            if self.confirm is None:
                return None
            self.robot.stop()
            if self.confirm(
                "Confirm the robot is on the opposite edge at the expected "
                "destination (motors are stopped)."
            ):
                return Localization(
                    node_id=None,
                    marker_id="operator-confirmed-destination",
                    confidence=1.0,
                    timestamp=self.clock(),
                    source="manual",
                )
            return None
        if monitor is not None:
            confirmer = LocalizationConfirmer(
                expected,
                policy=self._localization_policy,
                required=self.config.destination_confirmations,
                clock=self.clock,
                marker_to_node=self.marker_to_node,
            )
            while self.clock() < deadline:
                if monitor.requires_stop():
                    self.robot.stop()
                confirmer.offer(monitor.check(expected))
                if confirmer.complete:
                    return confirmer.latest
                self.sleep(self._period_s)
            return None
        if self.location_provider is not None:
            return self._confirm_location(expected, deadline)
        if self.confirm is not None:
            self.robot.stop()
            if self.confirm(
                f"Confirm the robot reached {expected} after crossing "
                "(motors are stopped)."
            ):
                return Localization(
                    node_id=expected,
                    marker_id=expected,
                    confidence=1.0,
                    timestamp=self.clock(),
                    source="manual",
                )
        return None

    def _run_maneuver(
        self,
        current: EdgeState,
        target: EdgeState,
        speed: int,
        deadline: float,
    ) -> tuple[bool, float]:
        """Drive the crossing profile until the target edge appears or time runs out."""
        started = self.clock()
        found = False
        self.detector.reset()
        phase_deadline = min(deadline, self._budget_deadline())
        while self.clock() < phase_deadline:
            if not self._motion_allowed():
                break
            wheels = self._maneuver_wheels(current, speed)
            try:
                self.robot.drive_wheels(wheels, context="switch")
            except (I2CError, SafetyError):
                return False, self.clock() - started
            self.sleep(max(0.0, min(self._period_s, phase_deadline - self.clock())))
            if not self._motion_allowed():
                break
            try:
                reading = self.robot.read_line_sensors()
            except (I2CError, SensorError):
                return False, self.clock() - started
            detection = self.detector.update(reading, now=self.clock())
            if detection.rejected:
                break
            if detection.edge is target:
                found = True
                break
        # `travelled_s` reports *moving* time only: the mandatory stop below is
        # cleanup, not part of the manoeuvre.
        motion_ended = self.clock()
        try:
            self.robot.stop()
        except I2CError:
            return False, motion_ended - started
        return found, motion_ended - started

    def _verify_edge(self, target: EdgeState, deadline: float) -> tuple[bool, int]:
        deadline = min(deadline, getattr(self, "_overall_deadline", math.inf))
        consecutive = 0
        previous_timestamp: float | None = None
        while self.clock() < deadline:
            try:
                reading = self.robot.read_line_sensors()
            except (I2CError, SensorError):
                return False, consecutive
            detection = self.detector.update(reading, now=self.clock())
            monotonic = (
                previous_timestamp is None or detection.timestamp > previous_timestamp
            )
            previous_timestamp = detection.timestamp
            if not monotonic:
                consecutive = 0
            elif detection.stable and detection.stable_edge is target:
                consecutive += 1
                if consecutive >= self.config.confirm_samples:
                    return True, consecutive
            else:
                consecutive = 0
            self.sleep(max(0.0, min(self._period_s, deadline - self.clock())))
        return False, consecutive

    def _resume(
        self, target: EdgeState, speed: int, deadline: float
    ) -> tuple[bool, float]:
        if self.config.resume_samples <= 0:
            return True, 0.0
        # Stationary verification has completed. Begin a fresh moving sample
        # sequence; time spent between phases is not a gap during tracking.
        self.detector.reset()
        started = self.clock()
        phase_deadline = min(deadline, self._budget_deadline())
        forward = max(1, int(speed * self.config.reduce_speed_factor))
        matches = 0
        for _ in range(self.config.resume_samples):
            if self.clock() >= phase_deadline or not self._motion_allowed():
                break
            try:
                self.robot.forward(forward)
            except (I2CError, SafetyError):
                return False, self.clock() - started
            self.sleep(max(0.0, min(self._period_s, phase_deadline - self.clock())))
            try:
                reading = self.robot.read_line_sensors()
            except (I2CError, SensorError):
                return False, self.clock() - started
            detection = self.detector.update(reading, now=self.clock())
            if not detection.rejected and detection.edge is target:
                matches += 1
            else:
                matches = 0
        try:
            self.robot.stop()
        except I2CError:
            return False, self.clock() - started
        return matches >= min(2, self.config.resume_samples), self.clock() - started

    # -- helpers ------------------------------------------------------------

    def _maneuver_wheels(
        self, current: EdgeState, speed: int
    ) -> tuple[int, int, int, int]:
        """Lateral strafe, or a forward/lateral blend for diagonal switching."""
        if self.config.mode == "lateral":
            return self._blend(0, current, speed, speed)
        angle = math.radians(self.config.crossing_angle_deg or 0.0)
        forward = max(1, int(round(speed * math.cos(angle))))
        lateral = int(round(speed * math.sin(angle)))
        return self._blend(forward, current, lateral, speed)

    @staticmethod
    def _blend(
        forward: int, current: EdgeState, lateral: int, limit: int
    ) -> tuple[int, int, int, int]:
        """Blend forward and lateral components inside ``limit`` counts.

        The forward and lateral components are scaled by the same factor, so the
        crossing angle is preserved while no wheel exceeds the calibrated cap.
        """
        if limit < 1:
            raise ConfigError(
                "switch speed ceiling is below one PWM count; the commanded "
                "crossing speed is not representable"
            )
        # The tape body lies on the side of the current edge, so crossing means
        # moving away from it: BLACK_LEFT strafes left, BLACK_RIGHT strafes right.
        if current is EdgeState.BLACK_LEFT:
            raw = (
                forward - lateral,
                forward + lateral,
                forward + lateral,
                forward - lateral,
            )
        else:
            raw = (
                forward + lateral,
                forward - lateral,
                forward - lateral,
                forward + lateral,
            )
        peak = max(abs(value) for value in raw)
        if peak > limit:
            scale = limit / peak
            raw = tuple(int(value * scale) for value in raw)
            if all(value == 0 for value in raw):
                # A ceiling too small to represent the manoeuvre is refused
                # rather than silently rounded up to a full count.
                raise ConfigError(
                    "switch speed ceiling is too small to represent the crossing"
                )
        return raw  # type: ignore[return-value]

    def _evaluate_geometry(self) -> dict[str, object] | None:
        if self.config.mode != "diagonal":
            return None
        if self.counts_to_mps is not None:
            # Validate against the commanded counts and the real polling rate.
            inputs = CrossingInputs.from_measured_command(
                self.geometry,
                speed_counts=max(1, int(self.config.speed)),
                counts_to_mps=float(self.counts_to_mps),
                rate_hz=float(self.config.poll_rate_hz),
            )
        else:
            inputs = CrossingInputs.from_config(self.geometry)
        evaluation = evaluate_crossing(
            inputs, float(self.config.crossing_angle_deg)
        )
        if not evaluation.unambiguous_geometry:
            raise ConfigError(
                "diagonal switch refused by geometry evaluation: "
                + "; ".join(evaluation.warnings[1:])
            )
        return evaluation.as_dict()

    def _fail(self, code: FaultCode, reason: str) -> None:
        """Record a failure without overwriting an earlier, more specific fault."""
        existing = self.robot.fault
        if existing is not None:
            # The first failure wins; append this context to it instead of
            # relabelling (for example) a sensor failure as a switch timeout.
            self.robot.safety.note_failure(f"{code.value}: {reason}")
            return
        try:
            self.robot.raise_fault(code, reason)
        except I2CError:
            pass

    def _shutdown(self) -> bool:
        """Mandatory stop before a result is reported. False when it failed."""
        try:
            self.robot.stop()
        except I2CError:
            return False
        return True

    def _motion_allowed(self) -> bool:
        """True while the robot is armed, fault-free, and not e-stopped."""
        return (
            self.robot.armed
            and self.robot.fault is None
            and not self.robot.emergency_stop_latched
        )

    def _budget_limit(self, *, switch_speed_candidate: int) -> float:
        """Allowed *moving* seconds for crossing + search + resume.

        Phase timeouts still apply on top. Physical switching additionally
        requires a measured travel budget in metres, converted to seconds with a
        conservative upper bound on the real speed (commanded counts times the
        measured m/s per count). The estimate bounds the manoeuvre; it is never
        used as evidence of arrival.
        """
        time_budget = min(float(self.config.max_travel_s), float(
            self.config.switch_timeout_s + self.config.search_timeout_s
        ))
        if not self.require_distance_bound:
            return time_budget
        worst_case_speed_mps = max(
            1, int(switch_speed_candidate)
        ) * float(self.counts_to_mps or 0.0)
        if worst_case_speed_mps <= 0:
            raise ConfigError("switch speed calibration is not positive")
        return min(
            time_budget,
            float(self.config.max_travel_m) / worst_case_speed_mps,
        )

    def _budget_deadline(self) -> float:
        """Wall-clock deadline for the *remaining* moving budget."""
        return self.clock() + max(0.0, self._budget_limit_s - self._moving_s)

    def _budget_exhausted(self) -> bool:
        return self._moving_s >= self._budget_limit_s - 1e-9

    def _result(
        self,
        completed: bool,
        reason: str,
        current: EdgeState,
        target: EdgeState,
        trace: _Trace,
        start: float,
        travelled: float,
        verification_samples: int,
        destination: Localization | None,
        geometry: dict[str, object] | None,
    ) -> SwitchResult:
        if self.telemetry is not None:
            last_phase = trace.phases[-1][0] if trace.phases else None
            for phase, _stamp in trace.phases:
                self.telemetry.switch_phase(
                    phase,
                    target_edge=target.value,
                    note=reason if phase == last_phase else None,
                )
            # Never claim the target edge was observed unless the switch really
            # completed and verified it.
            self.telemetry.control(
                event="switch_result",
                detected_edge=target.value if completed else None,
                target_edge=target.value,
                phase=last_phase,
                note=reason,
            )
        return SwitchResult(
            completed=completed,
            reason=reason,
            current_edge=current,
            target_edge=target,
            phases=tuple(trace.phases),
            elapsed_s=self.clock() - start,
            travelled_s=travelled,
            verification_samples=verification_samples,
            destination=destination,
            fault=self.robot.fault,
            geometry=geometry,
        )


__all__ = [
    "EdgeSwitcher",
    "SwitchAuthorization",
    "SwitchPhase",
    "SwitchResult",
]
