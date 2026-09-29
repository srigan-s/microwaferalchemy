"""Edge following: oriented boundary estimation, bounds, recovery, shutdown."""

from __future__ import annotations

import pytest

from waferbot import (
    FaultCode,
    MockI2CTransport,
    Robot,
    RobotConfig,
    SafetyError,
    SensorConfig,
)
from waferbot.localization import Localization, MockArrivalMonitor
from waferbot.navconfig import FollowConfig
from waferbot.sensing import (
    EdgeFollower,
    EdgeState,
    FollowStopReason,
    edge_byte_for,
    estimate_boundary,
    wheel_command,
)

LEFT = edge_byte_for(EdgeState.BLACK_LEFT)          # normalized 0100
RIGHT = edge_byte_for(EdgeState.BLACK_RIGHT)        # normalized 0010
ALL_WHITE = edge_byte_for(EdgeState.BOTH_WHITE)     # normalized 0000
ALL_BLACK = edge_byte_for(EdgeState.BOTH_BLACK)     # normalized 0110
STOP_PAYLOADS = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]


def norm_byte(*channels: int, config: SensorConfig | None = None) -> int:
    """Raw byte for normalised S1..S4 (BLACK = 1), respecting the config."""
    config = config or SensorConfig()
    assert len(channels) == 4
    raw = [0 if value else 1 for value in channels]
    return sum(
        bit << position for bit, position in zip(raw, config.bit_for_channel)
    )


def make_robot(sequence, *, clock, watchdog: bool = False, config=None):
    transport = MockI2CTransport(line_sensor_sequence=list(sequence))
    robot = Robot(
        transport, config or RobotConfig(), clock=clock, watchdog=watchdog
    )
    return robot, transport


# -- oriented boundary estimator --------------------------------------------


@pytest.mark.parametrize(
    ("pattern", "error", "confidence"),
    [
        ((1, 1, 0, 0), 0.0, 1.0),      # centred, tape on the left
        ((0, 1, 0, 0), 0.0, 1.0),      # centred, S2 only
        ((1, 0, 0, 0), -1.0, 1.0),     # boundary between S1 and S2
        ((1, 1, 1, 0), 1.0, 1.0),      # boundary between S3 and S4
        ((0, 1, 1, 0), 1.0, 1.0),      # narrow tape, boundary between S3/S4
        ((0, 0, 1, 0), 1.0, 1.0),      # narrow tape, S3 only
        ((1, 0, 1, 0), -1.0, 0.25),    # two black runs (contradictory)
        ((1, 0, 0, 1), -1.0, 0.25),    # two black blobs
        ((0, 1, 0, 1), 0.0, 0.25),     # two black runs, centred candidate
        ((1, 0, 1, 1), -1.0, 0.25),    # two black runs
        ((0, 0, 0, 1), 2.0, 0.35),     # boundary beyond S4
        ((0, 1, 1, 1), 2.0, 0.35),     # boundary beyond S4
        ((0, 0, 1, 1), 2.0, 0.35),     # boundary beyond S4
        ((1, 1, 1, 1), 2.0, 0.35),     # wide tape / junction
        ((0, 0, 0, 0), -2.0, 0.35),    # no tape: come back left
    ],
)
def test_black_left_estimates(pattern, error, confidence):
    estimate = estimate_boundary(pattern, EdgeState.BLACK_LEFT)
    assert estimate.error_pitches == pytest.approx(error)
    assert estimate.confidence == pytest.approx(confidence)
    assert estimate.edge is EdgeState.BLACK_LEFT


@pytest.mark.parametrize(
    ("pattern", "error", "confidence"),
    [
        ((0, 0, 1, 1), 0.0, 1.0),      # centred, tape on the right
        ((0, 0, 1, 0), 0.0, 1.0),      # centred, S3 only
        ((0, 0, 0, 1), 1.0, 1.0),      # boundary between S3 and S4
        ((0, 1, 1, 1), -1.0, 1.0),     # boundary between S1 and S2
        ((1, 0, 0, 0), -2.0, 0.35),    # boundary beyond S1
        ((1, 1, 0, 0), -2.0, 0.35),
        ((1, 1, 1, 0), -2.0, 0.35),
        ((1, 1, 1, 1), -2.0, 0.35),    # wide tape / junction
        ((0, 0, 0, 0), 2.0, 0.35),     # no tape: come back right
        ((0, 1, 0, 1), -1.0, 0.25),    # two black runs
        ((1, 0, 1, 0), 0.0, 0.25),     # two black runs
    ],
)
def test_black_right_estimates(pattern, error, confidence):
    estimate = estimate_boundary(pattern, EdgeState.BLACK_RIGHT)
    assert estimate.error_pitches == pytest.approx(error)
    assert estimate.confidence == pytest.approx(confidence)


def test_centred_patterns_are_exactly_zero_for_both_edges():
    for pattern in ((1, 1, 0, 0), (0, 1, 0, 0)):
        assert estimate_boundary(pattern, EdgeState.BLACK_LEFT).error_pitches == 0.0
    for pattern in ((0, 0, 1, 1), (0, 0, 1, 0)):
        assert estimate_boundary(pattern, EdgeState.BLACK_RIGHT).error_pitches == 0.0


def test_mirror_symmetry_across_edges():
    """Mirroring a *visible* boundary mirrors the estimate."""
    for pattern in [
        (1, 1, 0, 0),
        (0, 1, 0, 0),
        (1, 1, 1, 0),
        (0, 1, 1, 0),
        (0, 0, 1, 0),
        (1, 0, 0, 0),
    ]:
        mirrored = tuple(reversed(pattern))
        left = estimate_boundary(pattern, EdgeState.BLACK_LEFT)
        right = estimate_boundary(mirrored, EdgeState.BLACK_RIGHT)
        assert left.confidence == pytest.approx(1.0)
        assert right.confidence == pytest.approx(1.0)
        assert left.error_pitches == pytest.approx(-right.error_pitches)


def test_all_sixteen_patterns_produce_bounded_estimates():
    for bits in range(16):
        pattern = tuple((bits >> (3 - index)) & 1 for index in range(4))
        for edge in (EdgeState.BLACK_LEFT, EdgeState.BLACK_RIGHT):
            estimate = estimate_boundary(pattern, edge)
            assert estimate.error_pitches is not None
            assert abs(estimate.error_pitches) <= 2.0
            assert 0.0 <= estimate.confidence <= 1.0
            if estimate.confidence >= 0.6:
                # A confident estimate always has a located boundary.
                assert estimate.position is not None


def test_estimator_rejects_bad_input():
    with pytest.raises(ValueError):
        estimate_boundary((1, 1, 0, 0), EdgeState.BOTH_BLACK)
    with pytest.raises(Exception):
        estimate_boundary((1, 1, 0), EdgeState.BLACK_LEFT)
    with pytest.raises(Exception):
        estimate_boundary((1, 1, 0, 2), EdgeState.BLACK_LEFT)


def test_wheel_command_modes_and_bounds():
    differential = wheel_command(
        base_speed=40, correction=2, mode="differential", lateral_weight=0.5, limit=60
    )
    assert differential == (42, 42, 38, 38)
    lateral = wheel_command(
        base_speed=40, correction=2, mode="lateral", lateral_weight=0.5, limit=60
    )
    assert lateral == (42, 38, 38, 42)
    blended = wheel_command(
        base_speed=40, correction=2, mode="blended", lateral_weight=0.5, limit=60
    )
    assert blended == (42, 40, 38, 40)
    clamped = wheel_command(
        base_speed=40, correction=100, mode="differential", lateral_weight=0.5, limit=60
    )
    assert clamped == (60, 60, -60, -60)
    with pytest.raises(ValueError):
        wheel_command(
            base_speed=40, correction=1, mode="twist", lateral_weight=0.5, limit=60
        )


# -- loop behaviour ----------------------------------------------------------


def test_follow_requires_armed_robot(step_clock, no_sleep):
    robot, _transport = make_robot([LEFT] * 20, clock=step_clock)
    follower = EdgeFollower(robot, sleep=no_sleep)
    with pytest.raises(SafetyError):
        follower.follow_edge(EdgeState.BLACK_LEFT, max_iterations=3)


def test_follow_rejects_ambiguous_target(step_clock, no_sleep):
    robot, _transport = make_robot([LEFT] * 20, clock=step_clock)
    robot.arm()
    follower = EdgeFollower(robot, sleep=no_sleep)
    with pytest.raises(ValueError):
        follower.follow_edge(EdgeState.BOTH_BLACK, max_iterations=2)


def test_acquisition_failure_means_no_motion(step_clock, no_sleep):
    robot, transport = make_robot([ALL_WHITE] * 200, clock=step_clock)
    robot.arm()
    follower = EdgeFollower(robot, sleep=no_sleep)
    result = follower.follow_edge(EdgeState.BLACK_LEFT, max_iterations=5)

    assert result.stop_reason is FollowStopReason.ACQUISITION_FAILED
    assert result.acquired is False
    assert result.samples == 0
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.LINE_LOST
    # Every write is a stop: the motors were never commanded to move.
    assert transport.payloads[:4] == STOP_PAYLOADS
    assert all(payload[2] == 0 for payload in transport.payloads)


def test_follow_runs_to_iteration_bound_and_stops(step_clock, no_sleep):
    robot, transport = make_robot([LEFT] * 40, clock=step_clock)
    robot.arm()
    follower = EdgeFollower(robot, sleep=no_sleep)
    result = follower.follow_edge(EdgeState.BLACK_LEFT, max_iterations=5)

    assert result.stop_reason is FollowStopReason.MAX_ITERATIONS
    assert result.acquired is True
    assert result.samples == 5
    assert result.recoveries == 0
    assert result.final_detection is not None
    assert result.final_detection.edge is EdgeState.BLACK_LEFT
    assert transport.motor_payloads[-4:] == STOP_PAYLOADS
    assert robot.fault is None
    # A centred boundary commands four equal forward wheel speeds.
    assert transport.motor_payloads[:4] == STOP_PAYLOADS
    assert transport.motor_payloads[4:8] == [(0, 0, 40), (1, 0, 40), (2, 0, 40), (3, 0, 40)]


def test_off_centre_boundary_steers_toward_it(step_clock, no_sleep):
    """Boundary to the robot's right (BLACK_LEFT with 1110) steers right."""
    pattern = norm_byte(1, 1, 1, 0)
    robot, transport = make_robot([pattern] * 40, clock=step_clock)
    robot.arm()
    follower = EdgeFollower(robot, sleep=no_sleep)
    follower.follow_edge(EdgeState.BLACK_LEFT, max_iterations=4)
    left_speeds = [payload for payload in transport.motor_payloads[4:8]]
    # Differential correction to the right: left wheels faster than right.
    assert left_speeds[0][2] > left_speeds[2][2]
    assert left_speeds[1][2] > left_speeds[3][2]


def test_boundary_to_the_left_steers_left(step_clock, no_sleep):
    pattern = norm_byte(1, 0, 0, 0)  # boundary between S1 and S2
    robot, transport = make_robot([pattern] * 40, clock=step_clock)
    robot.arm()
    follower = EdgeFollower(robot, sleep=no_sleep)
    follower.follow_edge(EdgeState.BLACK_LEFT, max_iterations=4)
    payloads = transport.motor_payloads[4:8]
    assert payloads[0][2] < payloads[2][2]


def test_correction_is_slew_limited(step_clock, no_sleep):
    # Model fast clock reads, below the 250ms sensor sampling-gap limit.
    step_clock.step = 0.005
    """The command must not step by more than max_correction_delta per cycle."""
    pattern = norm_byte(1, 1, 1, 0)  # boundary between S3 and S4 (error +1)
    robot, transport = make_robot([pattern] * 40, clock=step_clock)
    robot.arm()
    follower = EdgeFollower(
        robot,
        config=FollowConfig(max_correction_delta=5, kp=40.0),
        sleep=no_sleep,
    )
    follower.follow_edge(EdgeState.BLACK_LEFT, max_iterations=4)
    groups = [transport.motor_payloads[i : i + 4] for i in range(4, 20, 4)]

    def correction(group):
        left = (group[0][2] + group[1][2]) / 2
        right = (group[2][2] + group[3][2]) / 2
        return left - right

    deltas = [
        correction(groups[index + 1]) - correction(groups[index])
        for index in range(len(groups) - 1)
    ]
    assert all(abs(delta) <= 10 for delta in deltas)


def test_large_steering_demand_reduces_forward_speed():
    config = FollowConfig(
        base_speed=40,
        max_correction=40,
        min_curve_speed_factor=0.4,
        curve_slowdown_start=0.2,
    )
    robot, _transport = make_robot([LEFT] * 10, clock=lambda: 1.0)
    follower = EdgeFollower(robot, config=config, sleep=lambda _: None)
    follower._curve_slow_samples = 0
    follower._minimum_base_command = config.base_speed
    assert follower._curve_speed(40, 4) == 40
    assert follower._curve_speed(40, 40) == 16
    assert follower._curve_slow_samples == 1
    assert follower._minimum_base_command == 16


def test_junction_slows_the_base_speed(step_clock, no_sleep):
    junction = norm_byte(1, 1, 1, 0)  # three black channels
    robot, transport = make_robot([LEFT] * 4 + [junction] * 6, clock=step_clock)
    robot.arm()
    follower = EdgeFollower(robot, sleep=no_sleep)
    follower.follow_edge(EdgeState.BLACK_LEFT, max_iterations=8)
    groups = [transport.motor_payloads[i : i + 4] for i in range(4, 36, 4)]

    def magnitude(group):
        return max(payload[2] for payload in group)

    # Junction samples (3 black channels) reduce the forward base speed.
    assert magnitude(groups[5]) < magnitude(groups[0])


def test_rejected_evidence_never_steers(step_clock, no_sleep):
    """A replayed/stale frame must not drive the motors."""
    robot, transport = make_robot([LEFT] * 40, clock=step_clock)
    robot.arm()
    follower = EdgeFollower(robot, sleep=no_sleep)
    # Feed one reading repeatedly with a frozen timestamp via the detector.
    reading = robot.read_line_sensors()
    detection = follower.detector.update(reading, now=step_clock())
    assert detection.rejected is False
    replayed = follower.detector.update(reading, now=step_clock())
    assert replayed.rejected is True


def test_edge_loss_recovers_and_resumes(step_clock, no_sleep):
    junction = norm_byte(0, 0, 0, 0)
    sequence = [LEFT] * 8 + [junction] * 25 + [LEFT] * 200
    robot, transport = make_robot(sequence, clock=step_clock)
    robot.arm()
    follower = EdgeFollower(
        robot, config=FollowConfig(recovery_search_s=6.0), sleep=no_sleep
    )
    result = follower.follow_edge(EdgeState.BLACK_LEFT, max_iterations=40)
    assert result.recoveries >= 1
    assert result.stop_reason is FollowStopReason.MAX_ITERATIONS
    assert transport.motor_payloads[-4:] == STOP_PAYLOADS


def test_line_lost_latches_fault_when_recovery_fails(step_clock, no_sleep):
    robot, transport = make_robot(
        [LEFT] * 8 + [ALL_WHITE] * 400, clock=step_clock
    )
    robot.arm()
    follower = EdgeFollower(
        robot,
        config=FollowConfig(recovery_enabled=False),
        sleep=no_sleep,
    )
    result = follower.follow_edge(EdgeState.BLACK_LEFT, max_iterations=60)
    assert result.stop_reason is FollowStopReason.LINE_LOST
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.LINE_LOST
    assert robot.armed is False
    assert transport.motor_payloads[-4:] == STOP_PAYLOADS


def test_recovery_ignores_a_flash_of_the_target_edge(step_clock, no_sleep):
    sequence = [LEFT] * 6 + [ALL_WHITE] * 8 + [LEFT] + [ALL_WHITE] * 200
    robot, _transport = make_robot(sequence, clock=step_clock)
    robot.arm()
    follower = EdgeFollower(robot, sleep=no_sleep)
    result = follower.follow_edge(EdgeState.BLACK_LEFT, max_iterations=80)
    assert result.stop_reason in (
        FollowStopReason.LINE_LOST,
        FollowStopReason.MAX_ITERATIONS,
    )


def test_persistent_wrong_edge_is_not_tracked_forever(step_clock, no_sleep):
    robot, _transport = make_robot([RIGHT] * 400, clock=step_clock)
    robot.arm()
    follower = EdgeFollower(
        robot,
        config=FollowConfig(max_wrong_edge_s=0.5, recovery_enabled=False),
        sleep=no_sleep,
    )
    result = follower.follow_edge(EdgeState.BLACK_LEFT, max_iterations=80)
    assert result.stop_reason is FollowStopReason.LINE_LOST
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.LINE_LOST


def test_arrival_monitor_ends_the_follow(step_clock, no_sleep):
    robot, _transport = make_robot([LEFT] * 40, clock=step_clock)
    robot.arm()
    follower = EdgeFollower(robot, sleep=no_sleep)
    monitor = MockArrivalMonitor(polls_before_arrival=2, clock=step_clock)
    result = follower.follow_edge(
        EdgeState.BLACK_LEFT,
        max_iterations=10,
        arrival_monitor=monitor,
        expected_node="B2",
    )
    assert result.stop_reason is FollowStopReason.ARRIVAL
    assert result.samples == 2
    assert monitor.queries == ["B2", "B2"]
    assert result.arrival_localization is not None
    assert result.arrival_localization.source == "mock"


def test_arrival_state_is_not_reused_between_invocations(step_clock, no_sleep):
    """A previous arrival must never satisfy a later follow call."""

    class ScriptedMonitor:
        """Returns scripted (node, timestamp) frames in order."""

        def __init__(self, frames):
            self.frames = list(frames)
            self.index = 0

        def requires_stop(self) -> bool:
            return False

        def check(self, expected_node: str):
            frame = self.frames[min(self.index, len(self.frames) - 1)]
            self.index += 1
            if frame is None:
                return None
            node, timestamp = frame
            if node != expected_node:
                return None
            return Localization(
                node_id=node,
                marker_id=node,
                confidence=1.0,
                timestamp=timestamp,
                source="scripted",
            )

    robot, _transport = make_robot([LEFT] * 120, clock=step_clock)
    robot.arm()
    follower = EdgeFollower(
        robot, config=FollowConfig(arrival_confirmations=2), sleep=no_sleep
    )
    start = step_clock()
    first_monitor = ScriptedMonitor(
        [("B2", start), ("B2", start + 0.1), ("B2", start + 0.2)]
    )
    first = follower.follow_edge(
        EdgeState.BLACK_LEFT,
        max_iterations=20,
        arrival_monitor=first_monitor,
        expected_node="B2",
    )
    assert first.stop_reason is FollowStopReason.ARRIVAL
    assert first.arrival_localization is not None
    assert first.arrival_localization.timestamp == start + 0.1

    # A second call that only ever re-offers the same frame must not inherit the
    # earlier confirmation: the confirmer has to be rebuilt per invocation.
    frozen = step_clock()
    second_monitor = ScriptedMonitor([("B2", frozen)] * 20)
    second = follower.follow_edge(
        EdgeState.BLACK_LEFT,
        max_iterations=5,
        arrival_monitor=second_monitor,
        expected_node="B2",
    )
    assert second.stop_reason is FollowStopReason.MAX_ITERATIONS
    assert second.samples == 5


def test_manual_arrival_is_requested_with_motors_stopped(step_clock, no_sleep):
    robot, transport = make_robot([LEFT] * 40, clock=step_clock)
    robot.arm()
    follower = EdgeFollower(robot, sleep=no_sleep)

    class StoppedMonitor:
        def requires_stop(self) -> bool:
            return True

        def check(self, expected_node: str):
            assert transport.motor_payloads[-4:] == STOP_PAYLOADS, (
                "operator confirmation must happen with the wheels stopped"
            )
            return Localization(
                node_id=expected_node,
                marker_id=expected_node,
                confidence=1.0,
                timestamp=step_clock(),
                source="manual",
            )

    result = follower.follow_edge(
        EdgeState.BLACK_LEFT,
        max_iterations=5,
        arrival_monitor=StoppedMonitor(),
        expected_node="B-",
    )
    assert result.stop_reason is FollowStopReason.ARRIVAL
    assert result.samples == 1


def test_sensor_failure_during_follow_latches_and_stops(step_clock, no_sleep):
    robot, transport = make_robot([LEFT] * 40, clock=step_clock)
    robot.arm()
    transport.line_sensor_length = 2  # malformed frame
    follower = EdgeFollower(robot, sleep=no_sleep)
    result = follower.follow_edge(EdgeState.BLACK_LEFT, max_iterations=5)
    assert result.stop_reason is FollowStopReason.FAULT
    assert robot.fault is not None
    assert robot.fault.code is FaultCode.SENSOR_FAILURE
    assert transport.motor_payloads[-4:] == STOP_PAYLOADS


def test_follow_caps_motor_magnitude(step_clock, no_sleep):
    pattern = norm_byte(0, 1, 1, 1)
    robot, transport = make_robot([pattern] * 40, clock=step_clock)
    robot.arm()
    follower = EdgeFollower(
        robot,
        config=FollowConfig(base_speed=100, max_correction=100, kp=50.0),
        sleep=no_sleep,
    )
    follower.follow_edge(EdgeState.BLACK_LEFT, max_iterations=3)
    assert all(
        payload[2] <= robot.config.motor.max_speed
        for payload in transport.motor_payloads
    )


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -1.0, 0.0, True])
def test_follow_rejects_invalid_durations_before_moving(step_clock, no_sleep, bad):
    robot, transport = make_robot([LEFT] * 40, clock=step_clock)
    robot.arm()
    follower = EdgeFollower(robot, sleep=no_sleep)
    with pytest.raises(ValueError):
        follower.follow_edge(EdgeState.BLACK_LEFT, max_duration_s=bad)
    assert transport.writes == []


@pytest.mark.parametrize("bad", [0, -3, 1.5, True])
def test_follow_rejects_invalid_iteration_counts(step_clock, no_sleep, bad):
    robot, transport = make_robot([LEFT] * 40, clock=step_clock)
    robot.arm()
    follower = EdgeFollower(robot, sleep=no_sleep)
    with pytest.raises(ValueError):
        follower.follow_edge(EdgeState.BLACK_LEFT, max_iterations=bad)
    assert transport.writes == []
