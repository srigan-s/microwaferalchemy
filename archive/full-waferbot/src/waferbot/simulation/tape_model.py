"""Deterministic kinematic tape model used as a closed-loop test bench.

What it models (all values are *synthetic assumptions*, not measurements):

* a tape band ``|y - centreline(x)| <= tape_width/2`` on the floor,
* four sensors on the chassis front, spaced ``sensor_pitch_m`` apart,
* a mecanum chassis whose velocity follows the vendored wheel-mixing signs
  (forward/lateral/yaw) with simple ``counts -> m/s``, ``counts -> rad/s``
  factors,
* quantised black/white sensor readings with optional bit-flip noise, dropouts,
  and injected I2C failures,
* a monotonic model clock that advances one control period per sensor read.

It implements the same ``I2CTransport`` surface as the real bus, so the real
``Robot``/``EdgeFollower`` drive it unchanged and the trajectory depends on the
controller's commands.
"""

from __future__ import annotations

import math
import random
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field

from ..config import RobotConfig, SensorConfig
from ..errors import I2CError
from ..hardware.sensors import decode_line_byte
from ..kinematics import DriveAction
from ..sensing.edge import EdgeState, estimate_boundary
from ..protocol import I2C_ADDRESS_DEFAULT, REG_LINE_SENSOR, REG_MOTOR


@dataclass(frozen=True)
class Pose:
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0


@dataclass
class TapeModelConfig:
    """Synthetic model parameters (documented assumptions, not measurements)."""

    #: Tape centreline ``y = centreline(x)``; a straight tape is the default.
    centreline: Callable[[float], float] = lambda x: 0.0
    tape_width_m: float = 0.030
    sensor_pitch_m: float = 0.012
    sensor_detection_width_m: float = 0.005
    sensor_forward_offset_m: float = 0.10
    #: Fraction of the sensor footprint that must be black.
    black_threshold: float = 0.5
    #: Synthetic kinematics: how fast one PWM count moves the chassis.
    counts_to_mps: float = 0.0020
    counts_to_rad_s: float = 0.010
    #: Control period simulated per sensor read.
    step_s: float = 0.05
    flip_probability: float = 0.0
    dropout_probability: float = 0.0
    seed: int = 0
    #: Test hook: invert the modelled lateral response so a closed-loop test can
    #: prove it is sensitive to the feedback sign. Never used in production.
    invert_lateral: bool = False
    #: Test hook: invert the modelled yaw response (the term a differential
    #: correction drives). Never used in production.
    invert_yaw: bool = False
    address: int = I2C_ADDRESS_DEFAULT
    register: int = REG_LINE_SENSOR
    sensor_config: SensorConfig = field(default_factory=SensorConfig)


@dataclass(frozen=True)
class TapeSample:
    time_s: float
    pose: Pose
    wheels: tuple[int, int, int, int]
    sensor_byte: int
    lateral_error_m: float
    heading_error_rad: float


@dataclass(frozen=True)
class ClosedLoopMetrics:
    samples: int
    following_samples: int
    rms_lateral_m: float | None
    max_lateral_m: float | None
    final_lateral_m: float | None
    rms_heading_rad: float | None
    final_heading_rad: float | None
    tail_estimate_rms_pitches: float | None
    tail_estimate_max_pitches: float | None
    final_estimate_error_pitches: float | None
    centred: bool
    converged: bool
    stop_reason: str
    travelled_m: float

    def as_dict(self) -> dict[str, object]:
        return {
            "samples": self.samples,
            "following_samples": self.following_samples,
            "rms_lateral_m": self.rms_lateral_m,
            "max_lateral_m": self.max_lateral_m,
            "final_lateral_m": self.final_lateral_m,
            "rms_heading_rad": self.rms_heading_rad,
            "final_heading_rad": self.final_heading_rad,
            "tail_estimate_rms_pitches": self.tail_estimate_rms_pitches,
            "tail_estimate_max_pitches": self.tail_estimate_max_pitches,
            "final_estimate_error_pitches": self.final_estimate_error_pitches,
            "centred": self.centred,
            "converged": self.converged,
            "stop_reason": self.stop_reason,
            "travelled_m": self.travelled_m,
        }


class TapeModelTransport:
    """``I2CTransport`` whose line-sensor byte comes from the tape model."""

    def __init__(
        self,
        config: TapeModelConfig | None = None,
        *,
        initial_pose: Pose | None = None,
    ) -> None:
        self.config = config or TapeModelConfig()
        self.pose = initial_pose or Pose()
        self.time_s = 0.0
        self.history: list[TapeSample] = []
        self.reads: list[tuple[int, int, int]] = []
        self.writes: list[tuple[int, int, tuple[int, ...]]] = []
        self.closed = False
        self._wheels = (0, 0, 0, 0)
        self._pending: dict[int, tuple[int, tuple[int, ...]]] = {}
        self._rng = random.Random(self.config.seed)
        #: Fail the next N sensor reads with an I2CError (deterministic).
        self.fail_reads_after: int | None = None
        self.read_attempts = 0

    # -- clock ---------------------------------------------------------------

    def clock(self) -> float:
        return self.time_s

    # -- I2CTransport protocol ----------------------------------------------

    def write_block(self, address: int, register: int, data: Sequence[int]) -> None:
        payload = tuple(int(value) for value in data)
        self.writes.append((address, register, payload))
        if register != REG_MOTOR or len(payload) != 3:
            return
        motor_id, direction, speed = payload
        signed = -speed if direction == 1 else speed
        self._pending[motor_id] = (motor_id, (signed,))
        if len(self._pending) == 4:
            self._wheels = tuple(
                self._pending[index][1][0] for index in range(4)
            )
            self._pending.clear()

    def read_block(self, address: int, register: int, length: int) -> list[int]:
        self.read_attempts += 1
        if (
            self.fail_reads_after is not None
            and self.read_attempts > self.fail_reads_after
        ):
            raise I2CError("tape model injected sensor read failure")
        self.reads.append((address, register, length))
        if register != self.config.register:
            # Only the line sensor is modelled; other registers read as zero.
            return [0] * max(0, length)
        self._advance(self.config.step_s)
        byte = self.sensor_byte()
        self.history.append(
            TapeSample(
                time_s=self.time_s,
                pose=self.pose,
                wheels=self._wheels,
                sensor_byte=byte,
                lateral_error_m=self.lateral_error_m(),
                heading_error_rad=self.heading_error_rad(),
            )
        )
        return [byte] * max(0, length)

    def close(self) -> None:
        self.closed = True

    # -- model ---------------------------------------------------------------

    @property
    def wheels(self) -> tuple[int, int, int, int]:
        return self._wheels

    def _advance(self, dt: float) -> None:
        w0, w1, w2, w3 = self._wheels
        forward = (w0 + w1 + w2 + w3) / 4.0
        # Vendor strafe_left is (-, +, +, -); +y is the robot's left.
        lateral = (-w0 + w1 + w2 - w3) / 4.0
        if self.config.invert_lateral:
            lateral = -lateral
        # Vendor rotate_left is (-, -, +, +); positive yaw is counter-clockwise.
        yaw = (-w0 - w1 + w2 + w3) / 4.0
        if self.config.invert_yaw:
            yaw = -yaw
        vx = forward * self.config.counts_to_mps
        vy = lateral * self.config.counts_to_mps
        omega = yaw * self.config.counts_to_rad_s
        heading = self.pose.heading
        self.pose = Pose(
            x=self.pose.x + (vx * math.cos(heading) - vy * math.sin(heading)) * dt,
            y=self.pose.y + (vx * math.sin(heading) + vy * math.cos(heading)) * dt,
            heading=heading + omega * dt,
        )
        self.time_s += dt

    # -- sensors -------------------------------------------------------------

    def _sensor_positions(self) -> list[tuple[float, float]]:
        """World positions of S1..S4 plus their footprint half width."""
        pitch = self.config.sensor_pitch_m
        forward = self.config.sensor_forward_offset_m
        offsets = (1.5 * pitch, 0.5 * pitch, -0.5 * pitch, -1.5 * pitch)
        heading = self.pose.heading
        positions = []
        for lateral in offsets:
            x = self.pose.x + forward * math.cos(heading) - lateral * math.sin(heading)
            y = self.pose.y + forward * math.sin(heading) + lateral * math.cos(heading)
            positions.append((x, y))
        return positions

    def _black_fraction(self, x: float, y: float) -> float:
        """Fraction of this sensor's footprint that lies on the black tape."""
        half_width = self.config.tape_width_m / 2.0
        half_detection = self.config.sensor_detection_width_m / 2.0
        centre = self.config.centreline(x)
        footprint_low = y - half_detection
        footprint_high = y + half_detection
        overlap = min(footprint_high, centre + half_width) - max(
            footprint_low, centre - half_width
        )
        if overlap <= 0.0:
            return 0.0
        return min(1.0, overlap / (2.0 * half_detection))

    def black_pattern(self) -> tuple[int, int, int, int]:
        """Ideal (noise-free) black/white channels S1..S4, 1 = black."""
        pattern = []
        for x, y in self._sensor_positions():
            fraction = self._black_fraction(x, y)
            pattern.append(1 if fraction >= self.config.black_threshold else 0)
        return tuple(pattern)  # type: ignore[return-value]

    def sensor_byte(self) -> int:
        """Pack the current (possibly noisy) channels the way the HAL expects."""
        black = list(self.black_pattern())
        if self.config.dropout_probability > 0 and (
            self._rng.random() < self.config.dropout_probability
        ):
            # A dropout looks like "no tape anywhere".
            black = [0, 0, 0, 0]
        elif self.config.flip_probability > 0:
            for index in range(4):
                if self._rng.random() < self.config.flip_probability:
                    black[index] = 1 - black[index]
        sensor_config = self.config.sensor_config
        raw = [
            (0 if value else 1) if sensor_config.black_is_raw_zero else value
            for value in black
        ]
        return sum(
            bit << position
            for bit, position in zip(raw, sensor_config.bit_for_channel)
        )

    # -- boundaries ----------------------------------------------------------

    def boundary_y(self, x: float, action: DriveAction | str) -> float:
        """World y of the boundary the robot is asked to follow."""
        edge = action.value if isinstance(action, DriveAction) else str(action)
        centre = self.config.centreline(x)
        half_width = self.config.tape_width_m / 2.0
        if edge.endswith("BLACK_LEFT") or edge == "black-left":
            # Black on the robot's left (+y): the followed boundary is the
            # band's smaller-y edge.
            return centre - half_width
        return centre + half_width

    def lateral_error_m(self, action: DriveAction | str = DriveAction.FORWARD) -> float:
        """Signed boundary offset: positive means "boundary to the robot's right"."""
        positions = self._sensor_positions()
        x_centre = (positions[0][0] + positions[3][0]) / 2.0
        y_centre = (positions[0][1] + positions[3][1]) / 2.0
        boundary = self.boundary_y(x_centre, action)
        # +y is the robot's left, so the follower's "steer right" sign is
        # `y_centre - boundary`.
        return y_centre - boundary

    def heading_error_rad(self) -> float:
        """Heading relative to the +x tape direction, wrapped to (-pi, pi]."""
        heading = self.pose.heading
        return (heading + math.pi) % (2 * math.pi) - math.pi


@dataclass
class ClosedLoopRun:
    result: object
    transport: TapeModelTransport
    metrics: ClosedLoopMetrics


def run_closed_loop(
    *,
    edge,
    follow_config=None,
    model_config: TapeModelConfig | None = None,
    initial_pose: Pose | None = None,
    iterations: int = 200,
    speed_pwm: int | None = None,
    robot_config: RobotConfig | None = None,
) -> ClosedLoopRun:
    """Drive the real ``EdgeFollower`` against the synthetic tape model."""
    from ..robot import Robot
    from ..sensing.edge import EdgeState
    from ..sensing.follower import EdgeFollower
    from ..sensing.position import DEFAULT_SENSOR_POSITIONS_MM
    from dataclasses import replace

    resolved_edge = (
        edge if isinstance(edge, EdgeState) else EdgeState.from_name(str(edge))
    )
    config = model_config or TapeModelConfig()
    if follow_config is None:
        from ..navconfig import FollowConfig
        follow_config = FollowConfig()
    if tuple(follow_config.sensor_positions_mm) == DEFAULT_SENSOR_POSITIONS_MM:
        # The synthetic bench specifies its own (uniform) sensor pitch. Give
        # the same positions to the estimator; physical runs retain the
        # measured, non-uniform layout from FollowConfig.
        half = config.sensor_pitch_m * 1000 / 2
        follow_config = replace(
            follow_config,
            sensor_positions_mm=(-3 * half, -half, half, 3 * half),
        )
    transport = TapeModelTransport(config, initial_pose=initial_pose)
    robot = Robot(
        transport,
        robot_config or RobotConfig(),
        clock=transport.clock,
        sleep=lambda _seconds: None,
        watchdog=False,
    )
    robot.arm()
    follower = EdgeFollower(
        robot, config=follow_config, clock=transport.clock, sleep=lambda _seconds: None
    )
    result = follower.follow_edge(
        resolved_edge,
        max_iterations=iterations,
        max_speed_pwm=speed_pwm,
    )
    metrics = _metrics(transport, resolved_edge, result)
    return ClosedLoopRun(result=result, transport=transport, metrics=metrics)


def _metrics(
    transport: TapeModelTransport, edge, result
) -> ClosedLoopMetrics:
    samples = transport.history
    following = [sample for sample in samples if any(sample.wheels)]

    def _rms(values):
        if not values:
            return None
        return math.sqrt(sum(value * value for value in values) / len(values))

    forward = transport.config.sensor_forward_offset_m
    edge_action = (
        "BLACK_LEFT" if edge.value == "BLACK_LEFT" else "BLACK_RIGHT"
    )
    lateral = []
    for sample in following:
        x_centre = sample.pose.x + forward * math.cos(sample.pose.heading)
        y_centre = sample.pose.y + forward * math.sin(sample.pose.heading)
        boundary = transport.boundary_y(x_centre, edge_action)
        lateral.append(abs(y_centre - boundary))
    headings = [abs(sample.heading_error_rad) for sample in following]
    travelled = 0.0
    if len(samples) >= 2:
        travelled = math.hypot(
            samples[-1].pose.x - samples[0].pose.x,
            samples[-1].pose.y - samples[0].pose.y,
        )
    final_lateral = lateral[-1] if lateral else None
    final_heading = headings[-1] if headings else None
    rms_lateral = _rms(lateral)
    # Re-estimate the boundary the controller saw from the recorded sensor bytes
    # so convergence is judged on the controller's own evidence, not on a hidden
    # model variable. Only the tail (after acquisition) is used.
    resolved_edge = (
        EdgeState.BLACK_LEFT if edge.value == "BLACK_LEFT" else EdgeState.BLACK_RIGHT
    )
    tail_errors: list[float] = []
    for sample in following[-40:]:
        reading = decode_line_byte(sample.sensor_byte, transport.config.sensor_config)
        estimate = estimate_boundary(reading.normalized, resolved_edge)
        if estimate.error_pitches is not None and estimate.confidence >= 0.6:
            tail_errors.append(estimate.error_pitches)
    tail_rms = _rms(tail_errors)
    tail_max = max((abs(value) for value in tail_errors), default=None)
    estimate_error = result.metrics.get("final_error_pitches")
    dead_band = transport.config.sensor_pitch_m
    centred = bool(tail_rms is not None and tail_rms <= 0.5)
    converged = bool(
        result.acquired
        and len(following) >= 20
        and centred
        and final_lateral is not None
        and final_lateral <= dead_band + transport.config.sensor_detection_width_m
    )
    return ClosedLoopMetrics(
        samples=len(samples),
        following_samples=len(following),
        rms_lateral_m=rms_lateral,
        max_lateral_m=max(lateral) if lateral else None,
        final_lateral_m=final_lateral,
        rms_heading_rad=_rms(headings),
        final_heading_rad=final_heading,
        tail_estimate_rms_pitches=tail_rms,
        tail_estimate_max_pitches=tail_max,
        final_estimate_error_pitches=(
            None if estimate_error is None else float(estimate_error)
        ),
        centred=centred,
        converged=converged,
        stop_reason=result.stop_reason.value,
        travelled_m=travelled,
    )


__all__ = [
    "ClosedLoopMetrics",
    "ClosedLoopRun",
    "Pose",
    "TapeModelConfig",
    "TapeModelTransport",
    "TapeSample",
    "run_closed_loop",
]
