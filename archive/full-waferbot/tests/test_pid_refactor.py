"""Geometry, quantized velocity, true PID, and high-rate polling contracts."""

from __future__ import annotations

import csv
import time

from scripts.compare_pid import simulate
from scripts.replay_ir_pid import replay

import pytest

from waferbot.config import RobotConfig
from waferbot.hardware.mock import MockI2CTransport
from waferbot.navconfig import FollowConfig
from waferbot.navconfig import NavConfig
from waferbot.robot import Robot
from waferbot.sensing.edge import EdgeState, edge_byte_for
from waferbot.sensing.follower import EdgeFollower, FollowStopReason
from waferbot.sensing.pid import PIDController, PIDGains
from waferbot.sensing.poller import LatestIRPoller
from waferbot.sensing.position import (
    EdgeVelocityEstimator, PositionEstimate, VelocityEstimate, estimate_position,
)


def position(edge_mm=0.0, ns=0, *, valid=True):
    return PositionEstimate(
        EdgeState.BLACK_LEFT, edge_mm if valid else None,
        -edge_mm if valid else None, -3.25 if valid else None,
        3.25 if valid else None, valid, 1.0 if valid else 0.0, ns, "test",
    )


def velocity(mm_s=0.0, ns=0, *, valid=True):
    return VelocityEstimate(mm_s if valid else None, valid, 1.0, ns, "test")


def test_all_sixteen_binary_patterns_do_not_invent_a_position():
    for mask in range(16):
        bits = tuple((mask >> (3 - i)) & 1 for i in range(4))
        for edge in (EdgeState.BLACK_LEFT, EdgeState.BLACK_RIGHT):
            measured = estimate_position(bits, edge, 123)
            if measured.valid:
                assert measured.lower_bound_mm <= measured.edge_mm <= measured.upper_bound_mm
                assert measured.robot_mm == -measured.edge_mm
            else:
                assert measured.edge_mm is None
                assert measured.robot_mm is None
    left = estimate_position((1, 1, 0, 0), EdgeState.BLACK_LEFT, 123)
    assert (left.edge_mm, left.lower_bound_mm, left.upper_bound_mm) == (0, -3.25, 3.25)
    assert estimate_position((0, 0, 1, 1), EdgeState.BLACK_RIGHT, 123).edge_mm == 0
    # The first off-centre pattern uses the inner sensor, while retaining the
    # entire interval of physically possible tape-edge positions.
    left_outer = estimate_position((1, 0, 0, 0), EdgeState.BLACK_LEFT, 123)
    assert (left_outer.edge_mm, left_outer.robot_mm) == (-3.25, 3.25)
    assert (left_outer.lower_bound_mm, left_outer.upper_bound_mm) == (-31.25, -3.25)
    assert estimate_position((1, 1, 1, 0), EdgeState.BLACK_LEFT, 123).robot_mm == -3.25
    assert estimate_position((0, 0, 0, 1), EdgeState.BLACK_RIGHT, 123).robot_mm == -3.25
    assert estimate_position((0, 1, 1, 1), EdgeState.BLACK_RIGHT, 123).robot_mm == 3.25
    layout_b = (-31.25, -17.25, 17.25, 31.25)
    assert estimate_position((1, 0, 0, 0), EdgeState.BLACK_LEFT, 123,
                             layout_b).robot_mm == 17.25
    partial = estimate_position((0, 0, 1, 0), EdgeState.BLACK_RIGHT, 123)
    assert partial.valid and partial.confidence < 1
    assert estimate_position((0, 1, 1, 1), EdgeState.BLACK_LEFT, 123).valid is False
    for uniform in ((0, 0, 0, 0), (1, 1, 1, 1)):
        assert not estimate_position(uniform, EdgeState.BLACK_LEFT, 123).valid


def test_velocity_uses_transition_time_and_ignores_repeated_bins():
    estimator = EdgeVelocityEstimator(tau_s=0.1, min_transition_s=0.01)
    assert estimator.update(position(0, 1_000_000_000)).robot_mm_s == 0
    assert estimator.update(position(0, 1_001_000_000)).robot_mm_s == 0
    assert estimator.update(position(3.25, 1_002_000_000)).reason == "transition too soon"
    moved = estimator.update(position(3.25, 1_100_000_000))
    assert moved.valid
    assert -250 <= moved.robot_mm_s < 0
    assert estimator.update(position(3.25, 1_101_000_000)).robot_mm_s == moved.robot_mm_s
    assert not estimator.update(position(0, 1_101_000_000)).valid
    assert not estimator.update(position(valid=False, ns=1_200_000_000)).valid


def test_pid_math_integrates_filters_by_velocity_and_prevents_windup():
    gains = PIDGains(kp=0.5, ki=0.2, kd=0.1, integral_limit_pwm=1,
                     max_correction_pwm=5, max_slew_pwm_per_s=100)
    controller = PIDController(gains)
    first = controller.update(reference_mm=0, measurement=position(2, 0),
                              velocity=velocity(-3, 0), timestamp_ns=0)
    assert first.error_mm == 2
    assert first.p_term == 1
    assert first.i_term == 0
    assert first.d_term == pytest.approx(0.3)
    second = controller.update(reference_mm=0, measurement=position(2, 100_000_000),
                               velocity=velocity(-3, 100_000_000), timestamp_ns=100_000_000)
    assert second.i_term == pytest.approx(0.04)
    assert second.output_raw == pytest.approx(1.34)
    # A setpoint step does not change D because it acts on measured velocity.
    third = controller.update(reference_mm=5, measurement=position(2, 200_000_000),
                              velocity=velocity(-3, 200_000_000), timestamp_ns=200_000_000)
    assert third.d_term == pytest.approx(second.d_term)
    for i in range(3, 30):
        out = controller.update(reference_mm=100, measurement=position(2, i * 100_000_000),
                                velocity=velocity(0, i * 100_000_000), timestamp_ns=i * 100_000_000)
        assert abs(out.output_pwm) <= 5
    assert controller.integral_pwm <= 1
    controller.note_motor_saturation()
    controller.reset()
    assert controller.integral_pwm == 0
    invalid = controller.update(reference_mm=0, measurement=position(valid=False, ns=4_000_000_000),
                                velocity=velocity(valid=False, ns=4_000_000_000), timestamp_ns=4_000_000_000)
    assert not invalid.measurement_valid and invalid.output_pwm == 0


def test_poller_retains_latest_sample_without_queue():
    transport = MockI2CTransport(line_sensor_byte=edge_byte_for(EdgeState.BLACK_LEFT))
    robot = Robot(transport, RobotConfig(), watchdog=False)
    poller = LatestIRPoller(robot, rate_hz=300)
    poller.start()
    try:
        first = poller.latest_after(0, 0.1)
        assert first.sequence >= 1
        time.sleep(0.025)
        latest = poller.latest_after(first.sequence, 0.1)
        assert latest.sequence > first.sequence
        assert latest.reading is not None
        assert latest.observed_hz is not None
        assert latest.error is None
    finally:
        poller.stop()
        robot.close()


def test_follower_control_csv_contains_pid_and_stops_on_invalid(tmp_path):
    cfg = FollowConfig(base_speed=5, kp_pwm_per_mm=0.5, kd_pwm_s_per_mm=0.01,
                       max_correction=5, recovery_enabled=False, edge_loss_samples=1)
    source = edge_byte_for(EdgeState.BLACK_LEFT)
    lost = edge_byte_for(EdgeState.BOTH_WHITE)
    transport = MockI2CTransport(line_sensor_sequence=[source] * 6 + [lost] * 3)
    robot = Robot(transport, RobotConfig().with_speed_limit(5), watchdog=False)
    robot.arm()
    path = tmp_path / "control.csv"
    follower = EdgeFollower(robot, config=cfg, sleep=lambda _: None,
                            control_log_csv=str(path))
    result = follower.follow_edge(EdgeState.BLACK_LEFT, max_iterations=20)
    assert result.stop_reason is FollowStopReason.LINE_LOST
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert rows and "p_term" in rows[0] and "motor_write_ms" in rows[0]
    assert all(int(payload[2]) <= 5 for _, _, payload in transport.writes)
    assert [payload for _, _, payload in transport.writes[-4:]] == [
        (index, 0, 0) for index in range(4)
    ]
    robot.close()


def test_offline_300_hz_polling_and_multiple_control_rates():
    for layout in ("a", "b"):
        for control_hz in (100, 200, 300):
            samples, _settling = simulate(layout=layout, kp=0.7, ki=0,
                                          kd=0.015, tau_s=0.05, poll_hz=300,
                                          control_hz=control_hz, duration_s=1)
            assert len(samples) == control_hz + 1
            assert all(max(map(abs, sample.wheels)) <= 5 for sample in samples)
            assert all(len(sample.pattern) == 4 for sample in samples)


def test_recorded_csv_replay_uses_only_actual_timestamps(tmp_path):
    input_path = tmp_path / "angle.csv"
    output_path = tmp_path / "replay.csv"
    patterns = ("1100", "1000", "1110", "0111", "0110", "0010", "0000")
    with input_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("read_end_ns", "black_s1", "black_s2", "black_s3", "black_s4"))
        for index, bits in enumerate(patterns):
            writer.writerow((1_000_000_000 + index * 10_000_000, *bits))
    count = replay(input_path, output_path, edge=EdgeState.BLACK_LEFT,
                   nav=NavConfig())
    assert count == len(patterns)
    with output_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert [row["normalized_ir"] for row in rows] == list(patterns)
    assert rows[1]["robot_mm"] == "3.25"
    assert rows[1]["error_mm"] == "-3.25"
    assert rows[-1]["valid"] == "False"
    assert rows[-1]["output_pwm"] == "0.0"
    assert rows[0]["read_end_ns"] == "1000000000"
