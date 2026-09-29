"""Continuous edge following on a selected, oriented tape boundary.

The controller steers from an *oriented boundary estimate* (see
:func:`waferbot.sensing.edge.estimate_boundary`): ``BLACK_LEFT`` follows the
black-to-white transition, ``BLACK_RIGHT`` the white-to-black one, and a
correctly centred pattern (``1100``/``0100`` or ``0011``/``0010``) gives exactly
zero error. There is no black-centroid averaging, so a wide tape or a junction
cannot drag the command sideways, and the controller never switches to the other
boundary: patterns showing only the opposite transition saturate with the sign
that steers back toward the followed edge.

Everything is in PWM counts. No value here is a velocity, and elapsed time is
never evidence about the tape.
"""

from __future__ import annotations

import math
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from enum import Enum

from ..errors import I2CError, SafetyError, SensorError
from ..localization import (
    Localization,
    LocalizationConfirmer,
    LocalizationPolicy,
)
from ..navconfig import FollowConfig
from ..safety import FaultCode, FaultRecord, SafetyState
from .edge import (
    BoundaryEstimate,
    CENTRE_POSITION,
    EdgeDetection,
    EdgeDetector,
    EdgeState,
    estimate_boundary,
)


class FollowStopReason(str, Enum):
    MAX_ITERATIONS = "MAX_ITERATIONS"
    MAX_DURATION = "MAX_DURATION"
    ARRIVAL = "ARRIVAL"
    STOP_REQUESTED = "STOP_REQUESTED"
    LINE_LOST = "LINE_LOST"
    ACQUISITION_FAILED = "ACQUISITION_FAILED"
    FAULT = "FAULT"


@dataclass(frozen=True)
class FollowResult:
    target_edge: EdgeState
    stop_reason: FollowStopReason
    samples: int
    elapsed_s: float
    recoveries: int
    final_detection: EdgeDetection | None
    fault: FaultRecord | None = None
    arrival_localization: Localization | None = None
    acquired: bool = True
    metrics: dict[str, float | int | str | None] = field(default_factory=dict)

    @property
    def completed(self) -> bool:
        return self.stop_reason in (
            FollowStopReason.ARRIVAL,
            FollowStopReason.MAX_ITERATIONS,
            FollowStopReason.MAX_DURATION,
        )

    def as_dict(self) -> dict[str, object]:
        return {
            "target_edge": self.target_edge.value,
            "stop_reason": self.stop_reason.value,
            "samples": self.samples,
            "elapsed_s": self.elapsed_s,
            "recoveries": self.recoveries,
            "acquired": self.acquired,
            "fault": self.fault.code.value if self.fault else None,
            "metrics": dict(self.metrics),
            "final_detection": (
                self.final_detection.as_dict() if self.final_detection else None
            ),
            "arrival_localization": (
                {
                    "node_id": self.arrival_localization.node_id,
                    "marker_id": self.arrival_localization.marker_id,
                    "confidence": self.arrival_localization.confidence,
                    "source": self.arrival_localization.source,
                    "timestamp": self.arrival_localization.timestamp,
                }
                if self.arrival_localization
                else None
            ),
        }


def wheel_command(
    *,
    base_speed: int,
    correction: float,
    mode: str,
    lateral_weight: float,
    limit: int,
) -> tuple[int, int, int, int]:
    """Blend a forward command with a differential and/or lateral correction."""
    correction = float(correction)
    if mode == "differential":
        differential, lateral = correction, 0.0
    elif mode == "lateral":
        differential, lateral = 0.0, correction
    elif mode == "blended":
        differential = correction * (1.0 - lateral_weight)
        lateral = correction * lateral_weight
    else:  # pragma: no cover - validated by config
        raise ValueError(f"unknown correction mode {mode!r}")

    raw = [
        base_speed + differential + lateral,
        base_speed + differential - lateral,
        base_speed - differential - lateral,
        base_speed - differential + lateral,
    ]
    wheels = []
    for value in raw:
        bounded = max(-limit, min(limit, int(round(value))))
        wheels.append(bounded)
    return tuple(wheels)  # type: ignore[return-value]


class EdgeFollower:
    """Follows one selected tape boundary using the four-channel sensor."""

    def __init__(
        self,
        robot,
        *,
        detector: EdgeDetector | None = None,
        config: FollowConfig | None = None,
        clock: Callable[[], float] | None = None,
        sleep: Callable[[float], None] = time.sleep,
        telemetry=None,
        marker_to_node: dict[str, str] | None = None,
    ) -> None:
        self.robot = robot
        self.config = (config or FollowConfig()).validate()
        self.clock = clock or getattr(robot, "clock", time.monotonic)
        self.sleep = sleep
        self.telemetry = telemetry
        self.marker_to_node = marker_to_node
        self.detector = detector or EdgeDetector(
            stable_samples=self.config.stable_samples,
            max_age_s=self.config.sensor_max_age_s,
            junction_channels=self.config.junction_channels,
            clock=self.clock,
        )
        self._period_s = 1.0 / float(self.config.rate_hz)
        self._localization_policy = LocalizationPolicy(
            max_age_s=self.config.localization_max_age_s,
            min_confidence=self.config.localization_min_confidence,
        )
        self._confirmers: dict[str, LocalizationConfirmer] = {}
        self.arrival_localization: Localization | None = None
        self._reset_controller()

    # -- public API ---------------------------------------------------------

    def follow_edge(
        self,
        target_edge: EdgeState | str,
        *,
        max_duration_s: float | None = None,
        max_iterations: int | None = None,
        arrival_monitor=None,
        expected_node: str | None = None,
        stop_event=None,
        max_speed_pwm: int | None = None,
    ) -> FollowResult:
        """Follow ``target_edge`` until a bound is reached.

        The controller acquires the boundary while stationary, then adjusts with
        a filtered PD law on the oriented boundary estimate, slows at junctions,
        recovers within bounds when the line is lost, and always stops the wheels
        before returning.
        """
        target = (
            target_edge
            if isinstance(target_edge, EdgeState)
            else EdgeState.from_name(target_edge)
        )
        if target not in (EdgeState.BLACK_LEFT, EdgeState.BLACK_RIGHT):
            raise ValueError(f"{target.value} is not a followable edge")
        if not self.robot.armed:
            raise SafetyError("edge following requires an armed robot")
        if max_speed_pwm is not None and (
            isinstance(max_speed_pwm, bool)
            or not isinstance(max_speed_pwm, int)
            or max_speed_pwm < 1
        ):
            raise ValueError("max_speed_pwm must be a positive integer count")

        duration_bound = (
            self.config.max_duration_s if max_duration_s is None else max_duration_s
        )
        if (
            isinstance(duration_bound, bool)
            or not isinstance(duration_bound, (int, float))
            or not math.isfinite(duration_bound)
            or duration_bound <= 0
        ):
            raise ValueError("max_duration_s must be a finite positive number")
        if max_iterations is not None and (
            isinstance(max_iterations, bool)
            or not isinstance(max_iterations, int)
            or max_iterations < 1
        ):
            raise ValueError("max_iterations must be a positive integer or None")

        robot_limit = self.robot.config.motor.max_speed
        base_speed = self.config.base_speed
        recovery_speed = self.config.recovery_speed
        if max_speed_pwm is not None:
            base_speed = min(base_speed, max_speed_pwm)
            recovery_speed = min(recovery_speed, max_speed_pwm)

        # Every invocation starts from a clean controller state.
        self._reset_state(target)
        start = self.clock()
        deadline = start + float(duration_bound)
        samples = 0
        recoveries = 0
        lost_samples = 0
        ambiguous_since: float | None = None
        wrong_edge_since: float | None = None
        rejected_since: float | None = None
        detection: EdgeDetection | None = None
        estimate: BoundaryEstimate | None = None
        stop_reason = FollowStopReason.MAX_DURATION
        acquired = False
        shutdown_ok = True
        if self.robot.state is not SafetyState.FAULT:
            self.robot.set_state(SafetyState.FOLLOWING)

        try:
            self.robot.stop()
            acquisition = self._acquire(
                target, deadline, max_speed_pwm=max_speed_pwm
            )
            if acquisition is None:
                if self.robot.fault is not None:
                    # Preserve the specific fault (for example SENSOR_FAILURE).
                    stop_reason = FollowStopReason.FAULT
                else:
                    stop_reason = FollowStopReason.ACQUISITION_FAILED
                    self._latch(
                        FaultCode.LINE_LOST,
                        f"could not acquire a confident {target.value} boundary "
                        "before any forward motion",
                    )
                lost_samples = 0
            else:
                acquired = True
                detection, estimate = acquisition

            while acquired:
                now = self.clock()
                if max_iterations is not None and samples >= max_iterations:
                    stop_reason = FollowStopReason.MAX_ITERATIONS
                    break
                if now >= deadline:
                    stop_reason = FollowStopReason.MAX_DURATION
                    break
                if stop_event is not None and stop_event.is_set():
                    stop_reason = FollowStopReason.STOP_REQUESTED
                    break
                if self.robot.fault is not None:
                    stop_reason = FollowStopReason.FAULT
                    break

                try:
                    reading = self.robot.read_line_sensors()
                except (I2CError, SensorError):
                    stop_reason = FollowStopReason.FAULT
                    break
                detection = self.detector.update(reading, now=self.clock())
                samples += 1
                self._log(detection, target, expected_node)

                arrival = self._check_arrival(arrival_monitor, expected_node)
                if arrival is not None:
                    stop_reason = FollowStopReason.ARRIVAL
                    break

                # Manual localization can take minutes. Never resume on the
                # reading taken before that stopped prompt, or after our bound.
                now = self.clock()
                if now >= deadline:
                    stop_reason = FollowStopReason.MAX_DURATION
                    break
                if arrival_monitor is not None and arrival_monitor.requires_stop():
                    try:
                        reading = self.robot.read_line_sensors()
                    except (I2CError, SensorError):
                        stop_reason = FollowStopReason.FAULT
                        break
                    detection = self.detector.update(reading, now=self.clock())
                    now = self.clock()
                    if now >= deadline:
                        stop_reason = FollowStopReason.MAX_DURATION
                        break

                if detection.rejected:
                    # Never steer from stale, replayed, or future evidence.
                    self._rejected_samples += 1
                    self.robot.stop()
                    if rejected_since is None:
                        rejected_since = now
                    if now - rejected_since > self.config.max_rejected_s:
                        stop_reason = self._handle_loss(
                            target, deadline, recoveries, recovery_speed,
                            "rejected sensor evidence", max_speed_pwm,
                        )
                        if stop_reason is None:
                            recoveries += 1
                            self._reset_controller()
                            rejected_since = None
                            self._sleep_remaining(now)
                            continue
                        break
                    self._sleep_remaining(now)
                    continue
                rejected_since = None

                estimate = estimate_boundary(detection.normalized, target)
                if estimate.error_pitches is not None:
                    self._record.append(estimate)
                if estimate.error_pitches is None:
                    lost_samples += 1
                    if lost_samples >= self.config.edge_loss_samples:
                        stop_reason = self._handle_loss(
                            target, deadline, recoveries, recovery_speed,
                            estimate.reason, max_speed_pwm,
                        )
                        if stop_reason is None:
                            recoveries += 1
                            lost_samples = 0
                            ambiguous_since = None
                            self._reset_controller()
                            self._sleep_remaining(now)
                            continue
                        break
                else:
                    lost_samples = 0

                if estimate.confidence < self.config.ambiguous_confidence:
                    if ambiguous_since is None:
                        ambiguous_since = now
                    if now - ambiguous_since > self.config.max_ambiguous_s:
                        stop_reason = self._handle_loss(
                            target, deadline, recoveries, recovery_speed,
                            f"ambiguous boundary readings ({estimate.reason})",
                            max_speed_pwm,
                        )
                        if stop_reason is None:
                            recoveries += 1
                            ambiguous_since = None
                            self._reset_controller()
                            self._sleep_remaining(now)
                            continue
                        break
                else:
                    ambiguous_since = None

                if detection.edge.is_edge and detection.edge is not target:
                    if wrong_edge_since is None:
                        wrong_edge_since = now
                    if now - wrong_edge_since > self.config.max_wrong_edge_s:
                        stop_reason = self._handle_loss(
                            target, deadline, recoveries, recovery_speed,
                            f"persistent {detection.edge.value} while following "
                            f"{target.value}",
                            max_speed_pwm,
                        )
                        if stop_reason is None:
                            recoveries += 1
                            wrong_edge_since = None
                            self._reset_controller()
                            self._sleep_remaining(now)
                            continue
                        break
                else:
                    wrong_edge_since = None

                correction = self._correction(
                    estimate.error_pitches, estimate.confidence, now
                )
                base = self._curve_speed(base_speed, correction)
                if detection.junction:
                    base = max(
                        1, int(round(base * self.config.junction_speed_factor))
                    )
                limit = min(robot_limit, base + self.config.max_correction)
                if max_speed_pwm is not None:
                    limit = min(limit, max_speed_pwm)
                wheels = wheel_command(
                    base_speed=base,
                    correction=correction,
                    mode=self.config.correction_mode,
                    lateral_weight=self.config.lateral_weight,
                    limit=limit,
                )
                try:
                    self.robot.drive_wheels(wheels, context="edge_follow")
                except (I2CError, SafetyError):
                    stop_reason = FollowStopReason.FAULT
                    break
                self._sleep_remaining(now)
        except (I2CError, SensorError, SafetyError) as exc:
            stop_reason = FollowStopReason.FAULT
            if self.robot.fault is None:
                code = (FaultCode.SENSOR_FAILURE if isinstance(exc, SensorError)
                        else FaultCode.MOTOR_COMMUNICATION_FAILURE)
                self._latch(code, str(exc))
        finally:
            try:
                self.robot.stop()
            except I2CError:
                # Robot.stop already latched the stop failure.
                shutdown_ok = False
            if self.robot.fault is None:
                self.robot.set_state(SafetyState.IDLE)

        if not shutdown_ok:
            # Success is only reported when the mandatory shutdown succeeded.
            stop_reason = FollowStopReason.FAULT
        return FollowResult(
            target_edge=target,
            stop_reason=stop_reason,
            samples=samples,
            elapsed_s=self.clock() - start,
            recoveries=recoveries,
            final_detection=detection,
            fault=self.robot.fault,
            arrival_localization=self.arrival_localization,
            acquired=acquired,
            metrics=self._metrics(),
        )

    # -- state ---------------------------------------------------------------

    def _reset_state(self, target: EdgeState) -> None:
        """Clear every piece of per-invocation state before following."""
        self.detector.reset()
        self._confirmers.clear()
        self.arrival_localization = None
        self._reset_controller()
        self._record: list[BoundaryEstimate] = []
        self._rejected_samples = 0
        self._ambiguous_samples = 0
        self._curve_slow_samples = 0
        self._minimum_base_command = self.config.base_speed
        self._target = target

    def _reset_controller(self) -> None:
        self._last_error: float | None = None
        self._last_error_time: float | None = None
        self._filtered_derivative = 0.0
        self._last_correction = 0.0

    def _metrics(self) -> dict[str, float | int | str | None]:
        errors = [
            estimate.error_pitches
            for estimate in getattr(self, "_record", [])
            if estimate.error_pitches is not None
        ]
        if errors:
            rms = math.sqrt(sum(error * error for error in errors) / len(errors))
            max_abs = max(abs(error) for error in errors)
            final = errors[-1]
        else:
            rms = max_abs = final = None
        return {
            "edge": self._target.value if getattr(self, "_target", None) else None,
            "boundary_samples": len(errors),
            "rms_error_pitches": rms,
            "max_abs_error_pitches": max_abs,
            "final_error_pitches": final,
            "rejected_samples": getattr(self, "_rejected_samples", 0),
            "ambiguous_samples": getattr(self, "_ambiguous_samples", 0),
            "curve_slow_samples": getattr(self, "_curve_slow_samples", 0),
            "minimum_base_command": getattr(
                self, "_minimum_base_command", self.config.base_speed
            ),
        }

    # -- phases --------------------------------------------------------------

    def _acquire(
        self, target: EdgeState, deadline: float, *, max_speed_pwm: int | None
    ) -> tuple[EdgeDetection, BoundaryEstimate] | None:
        """Stationary acquisition: prove the boundary before any forward motion."""
        acquisition_deadline = min(
            deadline, self.clock() + self.config.acquisition_timeout_s
        )
        consecutive = 0
        last: tuple[EdgeDetection, BoundaryEstimate] | None = None
        while self.clock() < acquisition_deadline:
            if self.robot.fault is not None or not self.robot.armed:
                return None
            try:
                reading = self.robot.read_line_sensors()
            except (I2CError, SensorError):
                return None
            detection = self.detector.update(reading, now=self.clock())
            if detection.rejected:
                consecutive = 0
                self._sleep_remaining(self.clock())
                continue
            estimate = estimate_boundary(detection.normalized, target)
            if (
                estimate.error_pitches is not None
                and estimate.confidence >= self.config.acquisition_confidence
            ):
                consecutive += 1
                last = (detection, estimate)
                if consecutive >= self.config.acquisition_samples:
                    self._record.append(estimate)
                    return last
            else:
                consecutive = 0
                self._record.append(estimate)
            self._sleep_remaining(self.clock())
        return None

    def _handle_loss(
        self,
        target: EdgeState,
        deadline: float,
        recoveries: int,
        recovery_speed: int,
        reason: str,
        max_speed_pwm: int | None,
    ) -> FollowStopReason | None:
        """Bounded recovery, or ``None`` when the caller should retry the loop."""
        if not self.config.recovery_enabled or recoveries >= self.config.recovery_attempts:
            self._latch(
                FaultCode.LINE_LOST,
                f"lost the {target.value} boundary: {reason}",
            )
            return FollowStopReason.LINE_LOST
        if self._recover(target, deadline, recovery_speed):
            return None
        self._latch(
            FaultCode.LINE_LOST,
            f"recovery failed to regain the {target.value} boundary: {reason}",
        )
        return FollowStopReason.LINE_LOST

    def _recover(self, target: EdgeState, deadline: float, speed: int) -> bool:
        """Bounded recovery: stop, back off, then search sideways for the edge.

        Every movement goes through :meth:`Robot.run_command`, which refreshes
        the command faster than the motion watchdog and always finally-stops.
        """
        direction = self._recovery_direction()
        try:
            self.robot.stop()
            reverse_deadline = min(
                deadline, self.clock() + self.config.recovery_reverse_s
            )
            if self.config.recovery_reverse_s > 0:
                self.robot.run_command(
                    lambda: self.robot.backward(speed),
                    self.config.recovery_reverse_s,
                    deadline=reverse_deadline,
                    context="follow-recovery-reverse",
                )
            strafe = self.robot.strafe_left if direction < 0 else self.robot.strafe_right
            search_deadline = min(
                deadline, self.clock() + self.config.recovery_search_s
            )
            while self.clock() < search_deadline:
                strafe(speed)
                self.sleep(self._period_s)
                reading = self.robot.read_line_sensors()
                detection = self.detector.update(reading, now=self.clock())
                if detection.rejected:
                    continue
                estimate = estimate_boundary(detection.normalized, target)
                if (
                    estimate.error_pitches is not None
                    and estimate.confidence >= self.config.acquisition_confidence
                ):
                    self.robot.stop()
                    self._record.append(estimate)
                    return True
            self.robot.stop()
            return False
        except (I2CError, SafetyError, SensorError):
            return False

    def _recovery_direction(self) -> float:
        if self._last_error is not None and abs(self._last_error) > 1e-9:
            return self._last_error
        return 0.0

    # -- control law ---------------------------------------------------------

    def _correction(self, error: float | None, confidence: float, now: float) -> float:
        if error is None:
            return float(self._last_correction)
        derivative_raw = 0.0
        if self._last_error is not None and self._last_error_time is not None:
            dt = now - self._last_error_time
            if dt > 0:
                derivative_raw = (error - self._last_error) / dt
        alpha = float(self.config.derivative_filter_alpha)
        self._filtered_derivative = (
            alpha * self._filtered_derivative + (1.0 - alpha) * derivative_raw
        )
        target = self.config.kp * error + self.config.kd * self._filtered_derivative
        # Low-confidence evidence steers more gently and never snaps the command.
        if confidence < 0.3:
            target *= 0.25
            self._ambiguous_samples += 1
        elif confidence < self.config.ambiguous_confidence:
            target *= 0.5
            self._ambiguous_samples += 1
        target = max(-float(self.config.max_correction), min(float(self.config.max_correction), target))
        delta = max(
            -float(self.config.max_correction_delta),
            min(float(self.config.max_correction_delta), target - self._last_correction),
        )
        correction = self._last_correction + delta
        self._last_error = error
        self._last_error_time = now
        self._last_correction = correction
        return correction

    def _curve_speed(self, base_speed: int, correction: float) -> int:
        """Slow forward travel as local steering demand rises."""
        maximum = float(self.config.max_correction)
        if maximum <= 0:
            return base_speed
        demand = min(1.0, abs(float(correction)) / maximum)
        start = float(self.config.curve_slowdown_start)
        if demand <= start:
            return base_speed
        blend = min(1.0, (demand - start) / max(1e-9, 1.0 - start))
        factor = 1.0 - blend * (1.0 - self.config.min_curve_speed_factor)
        command = max(1, int(round(base_speed * factor)))
        self._curve_slow_samples += 1
        self._minimum_base_command = min(self._minimum_base_command, command)
        return command

    # -- arrivals and faults -------------------------------------------------

    def _check_arrival(self, arrival_monitor, expected_node: str | None):
        if arrival_monitor is None or expected_node is None:
            return None
        confirmer = self._confirmers.get(expected_node)
        if confirmer is None:
            confirmer = LocalizationConfirmer(
                expected_node,
                policy=self._localization_policy,
                required=self.config.arrival_confirmations,
                clock=self.clock,
                marker_to_node=self.marker_to_node,
            )
            self._confirmers[expected_node] = confirmer
        if arrival_monitor.requires_stop():
            # Manual/operator confirmation must never be requested while moving.
            self.robot.stop()
        confirmer.offer(arrival_monitor.check(expected_node))
        if confirmer.complete:
            self.arrival_localization = confirmer.latest
            return confirmer.latest
        return None

    def _latch(self, code: FaultCode, message: str) -> None:
        """Latch a fault, or append to the existing one so it is never relabelled."""
        if self.robot.fault is not None:
            self.robot.safety.note_failure(f"{code.value}: {message}")
            return
        try:
            self.robot.raise_fault(code, message)
        except I2CError:
            pass

    # -- logging and pacing --------------------------------------------------

    def _log(self, detection: EdgeDetection, target: EdgeState, node: str | None) -> None:
        if self.telemetry is None:
            return
        # `node` is the *expected* destination, so it belongs in target_node.
        # The follower has no independent evidence of the current node.
        self.telemetry.control(
            event="follow",
            detected_edge=detection.edge.value,
            target_edge=target.value,
            current_node=None,
            target_node=node,
            stable=detection.stable,
            raw=detection.raw,
            normalized=detection.normalized,
            note=None if not detection.rejected else detection.reject_reason,
        )

    def _sleep_remaining(self, cycle_start: float) -> None:
        remaining = self._period_s - (self.clock() - cycle_start)
        if remaining > 0:
            self.sleep(remaining)


__all__ = [
    "CENTRE_POSITION",
    "EdgeFollower",
    "FollowResult",
    "FollowStopReason",
    "wheel_command",
]
