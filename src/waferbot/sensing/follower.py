"""One physical control loop: four IR bits -> position -> PID -> four wheels."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from ..errors import SensorError
from ..pidconfig import PIDConfig
from ..safety import SafetyState
from .control_log import ControlCSV
from .edge import EdgeState
from .pid import PIDController, PIDGains
from .poller import LatestIRPoller, PollSnapshot
from .position import EdgeVelocityEstimator, estimate_position


def wheel_command(base_pwm: int, correction: float, *, limit: int) -> tuple[int, int, int, int]:
    """Lateral mecanum mixer, with stateless integer rounding and a wheel cap."""
    targets = (
        base_pwm + correction, base_pwm - correction,
        base_pwm - correction, base_pwm + correction,
    )
    return tuple(max(-limit, min(limit, round(value))) for value in targets)


@dataclass(frozen=True)
class RunResult:
    stop_reason: str
    samples: int
    elapsed_s: float


class PIDFollower:
    def __init__(
        self,
        robot,
        config: PIDConfig,
        *,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        stop_requested: Callable[[], bool] = lambda: False,
        use_async_poller: bool = True,
    ) -> None:
        self.robot = robot
        self.config = config.validate()
        self.clock = clock
        self.sleep = sleep
        self.stop_requested = stop_requested
        self.use_async_poller = use_async_poller
        self.pid = PIDController(PIDGains(
            kp=config.kp, ki=config.ki, kd=config.kd,
            integral_limit_pwm=config.integral_limit_pwm,
            max_correction_pwm=config.max_correction_pwm,
            max_slew_pwm_per_s=config.max_slew_pwm_per_s,
            initial_dt_s=1 / config.rate_hz,
        ))
        self.velocity = EdgeVelocityEstimator(tau_s=config.velocity_filter_tau_s)

    def run(
        self, edge: EdgeState, *, duration_s: float = 3.0,
        log_csv: str | Path | None = None,
    ) -> RunResult:
        if not self.config.enabled:
            raise ValueError("PID controller is disabled in this config")
        if not self.robot.armed:
            raise ValueError("arm the robot before following")
        if edge not in (EdgeState.BLACK_LEFT, EdgeState.BLACK_RIGHT):
            raise ValueError("choose BLACK_LEFT or BLACK_RIGHT")
        if not math.isfinite(duration_s) or not 0 < duration_s <= 10:
            raise ValueError("duration_s must be between 0 and 10 seconds")

        period = 1 / self.config.rate_hz
        limit = min(self.config.max_pwm, self.robot.config.motor.max_speed)
        self.pid.reset()
        self.velocity.reset()
        started = self.clock()
        deadline = started + duration_s
        samples = 0
        reason = "MAX_DURATION"
        log = None
        poller = LatestIRPoller(self.robot, rate_hz=self.config.ir_polling_hz) if self.use_async_poller else None
        sequence = 0
        latest_poll: PollSnapshot | None = None

        def next_reading():
            nonlocal sequence, latest_poll
            if poller is None:
                reading = self.robot.read_line_sensors()
            else:
                previous = sequence
                snapshot = poller.latest_after(previous, period)
                if snapshot.error is not None:
                    raise SensorError(f"IR poller failed: {snapshot.error}") from snapshot.error
                if snapshot.sequence <= previous or snapshot.reading is None:
                    raise SensorError("no fresh IR sample before the control deadline")
                sequence = snapshot.sequence
                latest_poll = snapshot
                reading = snapshot.reading
            age = self.clock() - reading.timestamp
            if age < -0.01 or age > self.config.max_sample_age_s:
                raise SensorError(f"IR sample age {age:.3f}s exceeds limit")
            return reading

        self.robot.stop()
        try:
            if log_csv is not None:
                log = ControlCSV(log_csv)
            self.robot.set_state(SafetyState.FOLLOWING)
            if poller is not None:
                poller.start()
            # Acquire the selected edge while stationary before any motion.
            consecutive = 0
            acquisition_deadline = min(deadline, started + self.config.acquisition_timeout_s)
            while self.clock() < acquisition_deadline and consecutive < self.config.acquisition_samples:
                if self.stop_requested():
                    return RunResult("STOP_REQUESTED", samples, self.clock() - started)
                reading = next_reading()
                estimate = estimate_position(
                    reading.normalized, edge, int(reading.timestamp * 1e9),
                    self.config.sensor_positions_mm,
                )
                consecutive = consecutive + 1 if estimate.valid else 0
                self.sleep(period)
            if consecutive < self.config.acquisition_samples:
                return RunResult("ACQUISITION_FAILED", samples, self.clock() - started)

            lost = 0
            next_tick = self.clock()
            while self.clock() < deadline:
                if self.stop_requested():
                    reason = "STOP_REQUESTED"
                    break
                reading = next_reading()
                now = self.clock()
                position = estimate_position(
                    reading.normalized, edge, int(reading.timestamp * 1e9),
                    self.config.sensor_positions_mm,
                )
                velocity = self.velocity.update(position)
                if not position.valid or not velocity.valid:
                    self.robot.stop()
                    self.pid.reset()
                    lost += 1
                    if lost >= self.config.edge_loss_samples:
                        reason = "LINE_LOST"
                        break
                else:
                    lost = 0
                    output = self.pid.update(
                        reference_mm=self.config.reference_mm,
                        measurement=position, velocity=velocity,
                        timestamp_ns=position.timestamp_ns,
                    )
                    if not output.measurement_valid:
                        self.robot.stop()
                    else:
                        correction = output.output_pwm
                        base = self._curve_speed(correction)
                        wheels = wheel_command(base, correction, limit=limit)
                        if base + abs(correction) > limit:
                            self.pid.note_motor_saturation()
                        self.robot.drive_wheels(wheels, context="pid_follow")
                        samples += 1
                        if log is not None:
                            log.write(
                                timestamp_ns=position.timestamp_ns,
                                elapsed_s=now - started,
                                control_dt_s=output.dt_s,
                                raw_ir=f"0x{reading.raw_byte:02X}",
                                normalized_ir="".join(map(str, reading.normalized)),
                                selected_edge=edge.value,
                                edge_position_mm=position.edge_mm,
                                robot_position_mm=position.robot_mm,
                                velocity_mm_s=velocity.robot_mm_s,
                                measurement_valid=True,
                                reference_mm=self.config.reference_mm,
                                error_mm=output.error_mm,
                                p_term=output.p_term,
                                i_term=output.i_term,
                                d_term=output.d_term,
                                raw_correction=output.output_raw,
                                limited_correction=correction,
                                front_left_pwm=wheels[0], rear_left_pwm=wheels[1],
                                front_right_pwm=wheels[2], rear_right_pwm=wheels[3],
                                output_saturated=output.saturated or base + abs(correction) > limit,
                                safety_state=self.robot.state.value,
                                control_state="FOLLOWING",
                                polling_rate_hz=latest_poll.observed_hz if latest_poll else None,
                                poll_latency_ms=latest_poll.latency_ms if latest_poll else None,
                                missed_poll_deadlines=latest_poll.missed_deadlines if latest_poll else None,
                            )
                next_tick += period
                self.sleep(max(0.0, next_tick - self.clock()))
        finally:
            try:
                self.robot.stop()
            finally:
                if poller is not None:
                    poller.stop()
                if log is not None:
                    log.close()
                if self.robot.fault is None and self.robot.armed:
                    self.robot.set_state(SafetyState.IDLE)
        return RunResult(reason, samples, self.clock() - started)

    def _curve_speed(self, correction: float) -> int:
        demand = min(1.0, abs(correction) / self.config.max_correction_pwm) if self.config.max_correction_pwm else 0
        start = self.config.curve_slowdown_start
        if demand <= start:
            return self.config.base_pwm
        blend = min(1.0, (demand - start) / max(1e-9, 1.0 - start))
        factor = 1 - blend * (1 - self.config.min_curve_speed_factor)
        return max(1, round(self.config.base_pwm * factor))
