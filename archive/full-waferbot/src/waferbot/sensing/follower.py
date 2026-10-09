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
from collections.abc import Callable
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
from .pid import PIDController, PIDGains
from .poller import LatestIRPoller
from .position import EdgeVelocityEstimator, estimate_position
from .control_log import ControlCSV


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
        use_async_poller: bool = False,
        control_log_csv: str | None = None,
    ) -> None:
        self.robot = robot
        self.config = (config or FollowConfig()).validate()
        self.clock = clock or getattr(robot, "clock", time.monotonic)
        self.sleep = sleep
        self.telemetry = telemetry
        self.marker_to_node = marker_to_node
        self.use_async_poller = use_async_poller
        self.control_log_csv = control_log_csv
        self._control_log: ControlCSV | None = None
        self._poller: LatestIRPoller | None = None
        self._poll_sequence = 0
        self._poll_metrics: dict[str, float | int | None] = {}
        self.detector = detector or EdgeDetector(
            stable_samples=self.config.stable_samples,
            max_age_s=self.config.sensor_max_age_s,
            junction_channels=self.config.junction_channels,
            clock=self.clock,
        )
        self._period_s = 1.0 / float(self.config.rate_hz)
        centre_gap_mm = self.config.sensor_positions_mm[2] - self.config.sensor_positions_mm[1]
        self._pid = PIDController(PIDGains(
            kp=(self.config.kp_pwm_per_mm if self.config.kp_pwm_per_mm is not None
                else self.config.kp / centre_gap_mm),
            ki=self.config.ki_pwm_per_mm_s,
            kd=(self.config.kd_pwm_s_per_mm if self.config.kd_pwm_s_per_mm is not None
                else self.config.kd / centre_gap_mm),
            integral_limit_pwm=self.config.integral_limit_pwm,
            max_correction_pwm=self.config.max_correction,
            max_slew_pwm_per_s=(
                self.config.max_correction_slew_pwm_per_s
                if self.config.max_correction_slew_pwm_per_s is not None
                else self.config.max_correction_delta * 20.0
            ),
            initial_dt_s=1.0 / self.config.rate_hz,
        ))
        self._velocity = EdgeVelocityEstimator(tau_s=self.config.velocity_filter_tau_s)
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
        the geometry-aware PID law on the oriented boundary estimate, slows at junctions,
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
        if self.use_async_poller:
            self._poller = LatestIRPoller(self.robot, rate_hz=self.config.ir_polling_hz)
            self._poll_sequence = 0
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
            if self._poller is not None:
                self._poller.start()
            if self.control_log_csv is not None:
                self._control_log = ControlCSV(self.control_log_csv)
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
                    reading = self._read_line_sensors()
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
                        reading = self._read_line_sensors()
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
                position = estimate_position(
                    detection.normalized, target,
                    int(detection.timestamp * 1_000_000_000),
                    self.config.sensor_positions_mm,
                )
                if estimate.error_pitches is not None:
                    self._record.append(estimate)
                if not position.valid:
                    lost_samples += 1
                    if lost_samples >= self.config.edge_loss_samples:
                        stop_reason = self._handle_loss(
                            target, deadline, recoveries, recovery_speed,
                            position.reason, max_speed_pwm,
                        )
                        if stop_reason is None:
                            recoveries += 1
                            lost_samples = 0
                            ambiguous_since = None
                            self._reset_controller()
                            self._sleep_remaining(now)
                            continue
                        break
                    self._velocity.update(position)
                    self._pid.reset()
                    self.robot.stop()
                    if self._control_log is not None:
                        self._control_log.write(
                            timestamp_ns=int(now * 1e9), elapsed_s=now - start,
                            raw_ir=f"0x{reading.raw_byte:02X}",
                            normalized_ir="".join(map(str, reading.normalized)),
                            edge_state=detection.edge.value, selected_edge=target.value,
                            measurement_valid=False, control_state="INVALID_EDGE",
                            safety_state=self.robot.state.value,
                            front_left_pwm=0, rear_left_pwm=0,
                            front_right_pwm=0, rear_right_pwm=0,
                            polling_rate_hz=self._poll_metrics.get("polling_rate_hz"),
                            missed_poll_deadlines=self._poll_metrics.get("missed_poll_deadlines"),
                            skipped_ir_samples=self._poll_metrics.get("skipped_ir_samples"),
                        )
                    self._sleep_remaining(now)
                    continue
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

                velocity = self._velocity.update(position)
                computation_started_ns = time.monotonic_ns()
                control = self._pid.update(
                    reference_mm=self.config.reference_mm,
                    measurement=position,
                    velocity=velocity,
                    timestamp_ns=int(now * 1_000_000_000),
                )
                pid_compute_ms = (time.monotonic_ns() - computation_started_ns) / 1e6
                if not control.measurement_valid:
                    self.robot.stop()
                    self._sleep_remaining(now)
                    continue
                self._last_error = control.error_mm
                correction = control.output_pwm
                self._last_correction = correction
                self._last_control = control
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
                if base + abs(correction) > limit:
                    self._pid.note_motor_saturation()
                try:
                    motor_started_ns = time.monotonic_ns()
                    self.robot.drive_wheels(wheels, context="edge_follow")
                    motor_finished_ns = time.monotonic_ns()
                except (I2CError, SafetyError):
                    stop_reason = FollowStopReason.FAULT
                    break
                if self._control_log is not None:
                    period_ns = round(self._period_s * 1e9)
                    wall_dt_ns = None if self._last_control_wall_ns is None else (
                        motor_finished_ns - self._last_control_wall_ns
                    )
                    if wall_dt_ns is not None and wall_dt_ns > period_ns:
                        self._missed_control_deadlines += max(0, wall_dt_ns // period_ns - 1)
                    self._last_control_wall_ns = motor_finished_ns
                    self._control_log.write(
                        timestamp_ns=int(now * 1e9),
                        elapsed_s=now - start,
                        control_dt_s=control.dt_s,
                        raw_ir=f"0x{reading.raw_byte:02X}",
                        normalized_ir="".join(map(str, reading.normalized)),
                        edge_state=detection.edge.value,
                        selected_edge=target.value,
                        edge_position_mm=position.edge_mm,
                        robot_position_mm=position.robot_mm,
                        position_lower_bound_mm=position.lower_bound_mm,
                        position_upper_bound_mm=position.upper_bound_mm,
                        velocity_mm_s=velocity.robot_mm_s,
                        measurement_valid=control.measurement_valid,
                        measurement_age_ms=(now - reading.timestamp) * 1000,
                        reference_mm=self.config.reference_mm,
                        error_mm=control.error_mm,
                        kp_pwm_per_mm=self._pid.gains.kp,
                        ki_pwm_per_mm_s=self._pid.gains.ki,
                        kd_pwm_s_per_mm=self._pid.gains.kd,
                        p_term=control.p_term,
                        i_term=control.i_term,
                        d_term=control.d_term,
                        raw_correction=control.output_raw,
                        limited_correction=control.output_pwm,
                        front_left_pwm=wheels[0], rear_left_pwm=wheels[1],
                        front_right_pwm=wheels[2], rear_right_pwm=wheels[3],
                        output_saturated=control.saturated or base + abs(correction) > limit,
                        safety_state=self.robot.state.value,
                        control_state="FOLLOWING",
                        polling_rate_hz=self._poll_metrics.get("polling_rate_hz"),
                        control_rate_hz=(None if wall_dt_ns is None or wall_dt_ns <= 0
                                         else 1e9 / wall_dt_ns),
                        missed_poll_deadlines=self._poll_metrics.get("missed_poll_deadlines"),
                        skipped_ir_samples=self._poll_metrics.get("skipped_ir_samples"),
                        missed_control_deadlines=self._missed_control_deadlines,
                        poll_latency_ms=self._poll_metrics.get("poll_latency_ms"),
                        pid_compute_ms=pid_compute_ms,
                        motor_write_ms=(motor_finished_ns - motor_started_ns) / 1e6,
                        feedback_to_actuation_ms=(motor_finished_ns - int(reading.timestamp * 1e9)) / 1e6,
                    )
                self._sleep_remaining(now)
        except (I2CError, SensorError, SafetyError) as exc:
            stop_reason = FollowStopReason.FAULT
            if self.robot.fault is None:
                code = (FaultCode.SENSOR_FAILURE if isinstance(exc, SensorError)
                        else FaultCode.MOTOR_COMMUNICATION_FAILURE)
                self._latch(code, str(exc))
        finally:
            if self._poller is not None:
                self._poller.stop()
                self._poller = None
            if self._control_log is not None:
                self._control_log.close()
                self._control_log = None
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
        self._last_control_wall_ns: int | None = None
        self._missed_control_deadlines = 0
        self._poll_metrics = {}

    def _read_line_sensors(self):
        if self._poller is None:
            return self.robot.read_line_sensors()
        snapshot = self._poller.latest_after(self._poll_sequence, self._period_s)
        self._poll_metrics = {
            "polling_rate_hz": snapshot.observed_hz,
            "poll_latency_ms": snapshot.latency_ms,
            "missed_poll_deadlines": snapshot.missed_deadlines,
            "skipped_ir_samples": max(0, snapshot.sequence - self._poll_sequence - 1),
        }
        if snapshot.error is not None:
            raise SensorError(f"IR poller failed: {snapshot.error}")
        if snapshot.sequence <= self._poll_sequence or snapshot.reading is None:
            raise SensorError("no fresh IR sample before the control deadline")
        self._poll_sequence = snapshot.sequence
        return snapshot.reading

    def _reset_controller(self) -> None:
        self._pid.reset()
        self._velocity.reset()
        self._last_error: float | None = None
        self._last_correction = 0.0
        self._last_control = None

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
            **self._poll_metrics,
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
                reading = self._read_line_sensors()
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
                reading = self._read_line_sensors()
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
