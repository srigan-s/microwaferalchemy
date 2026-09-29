"""Closed-loop evidence for the real EdgeFollower on a synthetic tape model.

The model is a deterministic test bench, not a measurement of the robot: tape
geometry, sensor spacing, and the counts-to-m/s factors are explicit synthetic
assumptions (see ``waferbot.simulation.tape_model``). The trajectory depends on
the controller's commands, so these tests are real feedback evidence.
"""

from __future__ import annotations

import math

import pytest

from waferbot.navconfig import FollowConfig
from waferbot.sensing import EdgeState, estimate_boundary
from waferbot.simulation import (
    Pose,
    TapeModelConfig,
    TapeModelTransport,
    run_closed_loop,
)

#: Synthetic bench geometry: a 30 mm tape on a 20 mm sensor pitch. The width of
#: the "centred" dead-band is therefore one pitch (20 mm).
BENCH = TapeModelConfig(
    sensor_pitch_m=0.020,
    tape_width_m=0.030,
    sensor_detection_width_m=0.006,
)
TUNED = FollowConfig()  # kp=12, kd=0.5, slew=8, base=40 (tuned on this model)


def curved(amplitude_m: float = 0.02, period_m: float = 1.0) -> TapeModelConfig:
    return TapeModelConfig(
        sensor_pitch_m=0.020,
        tape_width_m=0.030,
        sensor_detection_width_m=0.006,
        centreline=lambda x: amplitude_m * math.sin(2 * math.pi * x / period_m),
    )


def noisy(flip: float = 0.08, dropout: float = 0.03, seed: int = 7) -> TapeModelConfig:
    return TapeModelConfig(
        sensor_pitch_m=0.020,
        tape_width_m=0.030,
        sensor_detection_width_m=0.006,
        flip_probability=flip,
        dropout_probability=dropout,
        seed=seed,
    )


# -- model mechanics ---------------------------------------------------------


def test_sensor_pattern_matches_the_pressed_boundary():
    """A boundary placed at the array centre reads as a centred pattern."""
    left = TapeModelTransport(BENCH, initial_pose=Pose(0.0, -0.015, 0.0))
    assert left.black_pattern() in ((1, 1, 0, 0), (0, 1, 0, 0))
    reading = estimate_boundary(left.black_pattern(), EdgeState.BLACK_LEFT)
    assert reading.error_pitches == 0.0

    right = TapeModelTransport(BENCH, initial_pose=Pose(0.0, 0.015, 0.0))
    assert right.black_pattern() in ((0, 0, 1, 1), (0, 0, 1, 0))
    assert estimate_boundary(right.black_pattern(), EdgeState.BLACK_RIGHT).error_pitches == 0.0


def test_off_tape_pose_reads_all_white():
    transport = TapeModelTransport(BENCH, initial_pose=Pose(0.0, 0.5, 0.0))
    assert transport.black_pattern() == (0, 0, 0, 0)
    # The HAL only uses the low nibble; all-white packs to 0x0F.
    assert transport.sensor_byte() == 0x0F


def test_wheel_command_moves_the_chassis_in_the_expected_direction():
    transport = TapeModelTransport(BENCH)
    # A complete forward command: four wheels forward at 50 counts.
    for motor_id in range(4):
        transport.write_block(0x2B, 0x01, [motor_id, 0, 50])
    transport.read_block(0x2B, 0x0A, 1)  # advances one control period
    assert transport.pose.x > 0.0
    assert transport.pose.y == pytest.approx(0.0, abs=1e-9)
    assert transport.pose.heading == pytest.approx(0.0, abs=1e-9)


def test_strafe_command_moves_laterally_and_rotation_turns():
    lateral = TapeModelTransport(BENCH)
    for motor_id, (direction, speed) in enumerate([(1, 50), (0, 50), (0, 50), (1, 50)]):
        lateral.write_block(0x2B, 0x01, [motor_id, direction, speed])
    lateral.read_block(0x2B, 0x0A, 1)
    assert lateral.pose.y > 0.0          # strafe left moves toward +y
    assert lateral.pose.x == pytest.approx(0.0, abs=1e-9)

    turning = TapeModelTransport(BENCH)
    for motor_id, (direction, speed) in enumerate([(1, 50), (1, 50), (0, 50), (0, 50)]):
        turning.write_block(0x2B, 0x01, [motor_id, direction, speed])
    turning.read_block(0x2B, 0x0A, 1)
    assert turning.pose.heading > 0.0    # rotate left is counter-clockwise


def test_model_is_deterministic():
    first = run_closed_loop(
        edge=EdgeState.BLACK_LEFT,
        model_config=BENCH,
        initial_pose=Pose(0.0, 0.005, 0.05),
        iterations=60,
    )
    second = run_closed_loop(
        edge=EdgeState.BLACK_LEFT,
        model_config=BENCH,
        initial_pose=Pose(0.0, 0.005, 0.05),
        iterations=60,
    )
    assert [sample.pose for sample in first.transport.history] == [
        sample.pose for sample in second.transport.history
    ]


def test_injected_sensor_failure_surfaces_as_i2c_error():
    from waferbot import I2CError

    transport = TapeModelTransport(BENCH)
    transport.fail_reads_after = 2
    transport.read_block(0x2B, 0x0A, 1)
    transport.read_block(0x2B, 0x0A, 1)
    with pytest.raises(I2CError):
        transport.read_block(0x2B, 0x0A, 1)


# -- closed-loop following ---------------------------------------------------


@pytest.mark.parametrize(
    ("edge", "y0", "heading"),
    [
        (EdgeState.BLACK_LEFT, 0.005, 0.05),
        (EdgeState.BLACK_LEFT, -0.030, -0.06),
        (EdgeState.BLACK_LEFT, 0.000, 0.00),
        (EdgeState.BLACK_RIGHT, -0.005, -0.05),
        (EdgeState.BLACK_RIGHT, 0.030, 0.06),
        (EdgeState.BLACK_RIGHT, 0.000, 0.00),
    ],
)
def test_straight_boundary_converges_for_both_edges(edge, y0, heading):
    run = run_closed_loop(
        edge=edge,
        model_config=BENCH,
        initial_pose=Pose(0.0, y0, heading),
        iterations=250,
        follow_config=TUNED,
    )
    metrics = run.metrics
    assert metrics.stop_reason == "MAX_ITERATIONS"
    assert run.result.acquired is True
    assert metrics.following_samples > 200
    assert metrics.travelled_m > 0.5, "the chassis must actually move"
    # Convergence is judged on the controller's own (quantised) evidence: the
    # boundary must settle inside the middle sensor gap.
    assert metrics.tail_estimate_rms_pitches is not None
    assert metrics.tail_estimate_rms_pitches <= 0.5
    assert metrics.centred is True
    # World residual is bounded by the one-pitch dead-band of a 4-channel array.
    assert metrics.final_lateral_m <= BENCH.sensor_pitch_m + BENCH.sensor_detection_width_m
    assert metrics.max_lateral_m <= 4 * BENCH.sensor_pitch_m


def test_follower_command_sign_matters_wrong_plant_diverges():
    """Inverting the modelled yaw response must make the loop diverge."""
    wrong_plant = TapeModelConfig(
        sensor_pitch_m=0.020,
        tape_width_m=0.030,
        sensor_detection_width_m=0.006,
        invert_yaw=True,
    )
    good = run_closed_loop(
        edge=EdgeState.BLACK_LEFT,
        model_config=BENCH,
        initial_pose=Pose(0.0, 0.005, 0.05),
        iterations=250,
        follow_config=TUNED,
    )
    bad = run_closed_loop(
        edge=EdgeState.BLACK_LEFT,
        model_config=wrong_plant,
        initial_pose=Pose(0.0, 0.005, 0.05),
        iterations=250,
        follow_config=TUNED,
    )
    assert good.metrics.centred is True
    assert bad.metrics.centred is False
    assert bad.metrics.stop_reason in {"LINE_LOST", "FAULT"}
    assert (bad.metrics.rms_lateral_m or 0.0) > 4 * (
        good.metrics.rms_lateral_m or 0.0
    )


def test_no_feedback_control_drifts_off_the_boundary():
    """With zero gains the robot drives straight and leaves the boundary."""
    run = run_closed_loop(
        edge=EdgeState.BLACK_LEFT,
        model_config=BENCH,
        initial_pose=Pose(0.0, -0.005, 0.15),
        iterations=250,
        follow_config=FollowConfig(kp=0.0, kd=0.0),
    )
    metrics = run.metrics
    assert metrics.following_samples > 0, "the run must actually move"
    assert metrics.centred is False
    assert metrics.max_lateral_m is not None
    assert metrics.max_lateral_m > 2 * BENCH.sensor_pitch_m


@pytest.mark.parametrize("edge", [EdgeState.BLACK_LEFT, EdgeState.BLACK_RIGHT])
def test_gentle_curve_is_tracked(edge):
    y0 = -0.015 if edge is EdgeState.BLACK_LEFT else 0.015
    run = run_closed_loop(
        edge=edge,
        model_config=curved(),
        initial_pose=Pose(0.0, y0, 0.0),
        iterations=500,
        follow_config=TUNED,
    )
    metrics = run.metrics
    assert metrics.stop_reason == "MAX_ITERATIONS"
    assert metrics.travelled_m > 1.0
    assert metrics.tail_estimate_rms_pitches is not None
    # One-pitch dead-band: the quantised tail error stays inside a single pitch.
    assert metrics.tail_estimate_rms_pitches <= 0.7
    assert metrics.max_lateral_m <= 1.5 * BENCH.sensor_pitch_m


@pytest.mark.parametrize("edge", [EdgeState.BLACK_LEFT, EdgeState.BLACK_RIGHT])
def test_windy_low_speed_profile_tracks_tighter_alternating_turns(edge):
    """A tighter S-curve exercises the adaptive curve speed and turn authority."""
    model = curved(amplitude_m=0.030, period_m=0.50)
    profile = FollowConfig(
        base_speed=24,
        kp=14.0,
        kd=0.6,
        max_correction=28,
        max_correction_delta=6,
        min_curve_speed_factor=0.4,
        curve_slowdown_start=0.15,
        recovery_enabled=False,
    )
    y0 = -0.015 if edge is EdgeState.BLACK_LEFT else 0.015
    run = run_closed_loop(
        edge=edge,
        model_config=model,
        initial_pose=Pose(0.0, y0, 0.0),
        iterations=300,
        follow_config=profile,
        speed_pwm=30,
    )
    assert run.metrics.stop_reason == "MAX_ITERATIONS"
    assert run.metrics.max_lateral_m <= 1.5 * BENCH.sensor_pitch_m
    assert run.result.metrics["curve_slow_samples"] > 100
    assert run.result.metrics["minimum_base_command"] < profile.base_speed


def test_old_fixed_speed_curve_profile_loses_the_tighter_path():
    """Control showing the windy-path scenario distinguishes the new profile."""
    old = FollowConfig(
        base_speed=25,
        kp=10.0,
        kd=0.3,
        max_correction=18,
        max_correction_delta=5,
        min_curve_speed_factor=1.0,
        recovery_enabled=False,
    )
    run = run_closed_loop(
        edge=EdgeState.BLACK_LEFT,
        model_config=curved(amplitude_m=0.030, period_m=0.50),
        initial_pose=Pose(0.0, -0.015, 0.0),
        iterations=300,
        follow_config=old,
        speed_pwm=30,
    )
    assert run.metrics.stop_reason == "LINE_LOST"


@pytest.mark.parametrize("edge", [EdgeState.BLACK_LEFT, EdgeState.BLACK_RIGHT])
def test_five_count_windy_profile_never_exceeds_the_motor_limit(edge):
    from dataclasses import replace
    from waferbot.config import RobotConfig
    from waferbot.navconfig import NavConfig

    robot = RobotConfig.load_json("config/robot.first-run.json")
    nav = NavConfig.load_json("config/nav.windy-first-run.json")
    assert robot.motor.max_speed == 5
    y0 = -0.015 if edge is EdgeState.BLACK_LEFT else 0.015
    run = run_closed_loop(
        edge=edge,
        model_config=curved(amplitude_m=0.020, period_m=0.15),
        initial_pose=Pose(0.0, y0, 0.0),
        iterations=1200,
        follow_config=replace(nav.follow, max_duration_s=120.0),
        robot_config=robot,
    )
    assert run.metrics.stop_reason == "MAX_ITERATIONS"
    assert run.metrics.travelled_m > 0.30
    assert run.result.metrics["curve_slow_samples"] > 0
    assert all(
        len(payload) != 3 or payload[2] <= 5
        for _address, _register, payload in run.transport.writes
    )
    assert run.transport.wheels == (0, 0, 0, 0)


def test_five_count_first_run_profile_loses_the_same_windy_curve():
    from dataclasses import replace
    from waferbot.config import RobotConfig
    from waferbot.navconfig import NavConfig

    run = run_closed_loop(
        edge=EdgeState.BLACK_LEFT,
        model_config=curved(amplitude_m=0.020, period_m=0.15),
        initial_pose=Pose(0.0, -0.015, 0.0),
        iterations=1200,
        follow_config=replace(
            NavConfig.load_json("config/nav.first-run.json").follow,
            max_duration_s=120.0,
        ),
        robot_config=RobotConfig.load_json("config/robot.first-run.json"),
    )
    assert run.metrics.stop_reason == "LINE_LOST"


@pytest.mark.parametrize("edge", [EdgeState.BLACK_LEFT, EdgeState.BLACK_RIGHT])
def test_quantisation_noise_and_dropouts_stay_bounded(edge):
    run = run_closed_loop(
        edge=edge,
        model_config=noisy(),
        initial_pose=Pose(0.0, 0.005 if edge is EdgeState.BLACK_LEFT else -0.005, 0.05),
        iterations=500,
        follow_config=TUNED,
    )
    metrics = run.metrics
    assert metrics.stop_reason == "MAX_ITERATIONS"
    assert metrics.travelled_m > 1.0
    assert metrics.tail_estimate_rms_pitches is not None
    assert metrics.tail_estimate_rms_pitches <= 0.7
    assert metrics.max_lateral_m <= 4 * BENCH.sensor_pitch_m


def test_missing_tape_prevents_any_motion():
    """Off the tape the follower must acquire nothing and never drive."""
    run = run_closed_loop(
        edge=EdgeState.BLACK_LEFT,
        model_config=BENCH,
        initial_pose=Pose(0.0, 0.30, 0.0),
        iterations=100,
        follow_config=TUNED,
    )
    assert run.result.acquired is False
    assert run.metrics.following_samples == 0
    assert run.metrics.travelled_m < 1e-6
    assert run.result.stop_reason.value == "ACQUISITION_FAILED"


def test_boundary_outside_the_array_prevents_motion():
    """A visible tape whose boundary is outside the sensor span must not drive."""
    run = run_closed_loop(
        edge=EdgeState.BLACK_LEFT,
        model_config=BENCH,
        initial_pose=Pose(0.0, 0.060, 0.0),
        iterations=100,
        follow_config=TUNED,
    )
    assert run.result.acquired is False
    assert run.metrics.following_samples == 0
    assert run.metrics.travelled_m < 1e-6


def test_short_run_stops_the_wheels():
    run = run_closed_loop(
        edge=EdgeState.BLACK_LEFT,
        model_config=BENCH,
        initial_pose=Pose(0.0, 0.0, 0.0),
        iterations=5,
        follow_config=TUNED,
    )
    # The final command must be a stop: no wheel is left driving.
    assert run.transport.wheels == (0, 0, 0, 0)
    assert run.metrics.stop_reason == "MAX_ITERATIONS"


@pytest.mark.parametrize("edge", [EdgeState.BLACK_LEFT, EdgeState.BLACK_RIGHT])
def test_sensor_read_failure_stops_the_closed_loop(edge):
    from waferbot import Robot, FaultCode
    from waferbot.sensing.follower import EdgeFollower

    transport = TapeModelTransport(
        BENCH, initial_pose=Pose(0.0, -0.015 if edge is EdgeState.BLACK_LEFT else 0.015, 0.0)
    )
    transport.fail_reads_after = 20
    robot = Robot(transport, clock=transport.clock, sleep=lambda _: None, watchdog=False)
    try:
        robot.arm()
        result = EdgeFollower(robot, config=TUNED, clock=transport.clock,
                              sleep=lambda _: None).follow_edge(edge, max_iterations=200)
        assert any(any(sample.wheels) for sample in transport.history)
        assert transport.read_attempts == 21
        assert result.stop_reason.value == "FAULT"
        assert result.fault.code is FaultCode.SENSOR_FAILURE
        assert transport.wheels == (0, 0, 0, 0)
        assert [payload for _, _, payload in transport.writes[-4:]] == [(i, 0, 0) for i in range(4)]
    finally:
        robot.close()
    assert transport.closed
