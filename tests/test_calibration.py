"""Calibration utilities: wheels, sensors, crossing sweep, and speed maths."""

from __future__ import annotations

import csv

import pytest

from waferbot import MockI2CTransport, Robot, RobotConfig, SafetyError
from waferbot.calibration import (
    CrossingSweep,
    SpeedCalibrationReport,
    SpeedSample,
    WheelCalibrator,
    calibrate_channels,
    compute_counts_to_mps,
    format_samples,
    read_samples,
    speed_calibration_instructions,
)
from waferbot.sensing import EdgeState, edge_byte_for

STOP_PAYLOADS = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]


def make_robot(sequence=None, *, clock):
    transport = MockI2CTransport(line_sensor_sequence=list(sequence or []))
    robot = Robot(transport, RobotConfig(), clock=clock, watchdog=False)
    return robot, transport


# -- wheels ------------------------------------------------------------------


def test_wheel_probe_confirms_and_drives_one_wheel(clock, no_sleep):
    robot, transport = make_robot(clock=clock)
    robot.arm()
    calibrator = WheelCalibrator(robot, ask=lambda _p: "yes", sleep=no_sleep)
    probe = calibrator.probe(2, speed=30, duration_s=0.1)

    assert probe.confirmed is True
    # Only motor 2 was driven, then everything stopped.
    assert transport.payloads[:4] == [(0, 0, 0), (1, 0, 0), (2, 0, 30), (3, 0, 0)]
    assert transport.payloads[-4:] == STOP_PAYLOADS


def test_full_wheel_run_marks_uncertain_and_suggests_inversion(clock, no_sleep):
    robot, _transport = make_robot(clock=clock)
    robot.arm()
    answers = iter(["yes", "no", "yes", "yes", "yes", "yes", "yes", "yes", "yes", "yes"])
    calibrator = WheelCalibrator(robot, ask=lambda _p: next(answers), sleep=no_sleep)
    report = calibrator.run(speed=20, duration_s=0.05)
    assert report.confirmed is False
    assert report.suggested_invert == [1]
    payload = report.as_dict()
    assert payload["uncertain_until_confirmed"] is True
    assert len(payload["probes"]) == 4
    assert payload["direction_checks"]["forward"] == "yes"
    assert payload["confirmed"] is False


def test_partial_wheel_run_is_not_a_global_verification(clock, no_sleep):
    robot, _transport = make_robot(clock=clock)
    robot.arm()
    calibrator = WheelCalibrator(robot, ask=lambda _p: "yes", sleep=no_sleep)
    report = calibrator.run(speed=20, duration_s=0.01, wheels=[0], directions=False)
    assert report.probes  # the wheel itself was checked
    assert report.all_wheels_tested is False
    assert report.confirmed is False


def test_wheel_calibration_requires_arming(clock, no_sleep):
    robot, _transport = make_robot(clock=clock)
    calibrator = WheelCalibrator(robot, ask=lambda _p: "yes", sleep=no_sleep)
    with pytest.raises(SafetyError):
        calibrator.probe(0, speed=20, duration_s=0.01)


def test_wheel_calibration_rejects_bad_wheel_id(clock, no_sleep):
    robot, _transport = make_robot(clock=clock)
    robot.arm()
    calibrator = WheelCalibrator(robot, ask=lambda _p: "yes", sleep=no_sleep)
    with pytest.raises(ValueError):
        calibrator.run(wheels=[9])


def test_wheel_probe_survives_an_enabled_watchdog(no_sleep):
    """A probe longer than the watchdog timeout must refresh, not trip it."""
    from waferbot import RobotConfig, SafetyConfig

    transport = MockI2CTransport()
    config = RobotConfig(
        safety=SafetyConfig(motion_timeout_s=0.1, watchdog_period_s=0.005)
    )
    robot = Robot(transport, config)
    try:
        robot.arm()
        calibrator = WheelCalibrator(robot, ask=lambda _p: "yes", sleep=no_sleep)
        probe = calibrator.probe(1, speed=20, duration_s=0.5)
        assert probe.confirmed is True
        assert robot.fault is None, "watchdog fired during a refreshing probe"
        assert transport.payloads[-4:] == STOP_PAYLOADS
    finally:
        robot.close()


# -- sensors -----------------------------------------------------------------


def test_sensor_samples_and_formatting(clock, no_sleep):
    robot, _transport = make_robot([edge_byte_for(EdgeState.BLACK_LEFT)] * 3, clock=clock)
    samples = read_samples(robot, 3, interval_s=0.0, sleep=no_sleep)
    assert len(samples) == 3
    text = format_samples(samples)
    assert "raw_byte" in text
    assert "0x07" in text


def test_channel_calibration_recovers_bit_order(clock, no_sleep):
    # Baseline (all white) first, then one channel covered at a time. Raw 0
    # means black, so covering S1 clears bit 2, S2 clears bit 3, and so on.
    per_channel_bytes = [
        0b1011,  # black under S1 -> bit 2 reads 0
        0b0111,  # black under S2 -> bit 3
        0b1101,  # black under S3 -> bit 1
        0b1110,  # black under S4 -> bit 0
    ]
    sequence = [0b1111] * 5 + [
        byte for byte in per_channel_bytes for _ in range(5)
    ]
    robot, _transport = make_robot(sequence, clock=clock)
    report = calibrate_channels(
        robot,
        ask=lambda _p: "",
        samples_per_channel=5,
        interval_s=0.0,
        sleep=no_sleep,
    )
    assert report.confirmed is True
    assert report.suggested_bit_for_channel == [2, 3, 1, 0]
    assert report.suggested_black_is_raw_zero is True
    assert report.channel_direction["S1 (leftmost)"] == "black_is_raw_zero"
    assert len(report.samples) == 20


def test_channel_calibration_reports_incomplete_runs(clock, no_sleep):
    robot, _transport = make_robot([0xFF] * 60, clock=clock)
    answers = iter(["", "skip", "skip", "skip", "skip"])
    report = calibrate_channels(
        robot,
        ask=lambda _p: next(answers),
        samples_per_channel=5,
        interval_s=0.0,
        sleep=no_sleep,
    )
    assert report.confirmed is False
    assert report.suggested_bit_for_channel is None
    assert report.notes


def test_channel_calibration_detects_inverted_polarity(clock, no_sleep):
    """Raw 1 means black: covering a channel must set its bit."""
    per_channel_bytes = [
        0b0100,  # S1 -> bit 2 set
        0b1000,  # S2 -> bit 3 set
        0b0010,  # S3 -> bit 1 set
        0b0001,  # S4 -> bit 0 set
    ]
    sequence = [0b0000] * 5 + [
        byte for byte in per_channel_bytes for _ in range(5)
    ]
    robot, _transport = make_robot(sequence, clock=clock)
    report = calibrate_channels(
        robot, ask=lambda _p: "", samples_per_channel=5, interval_s=0.0, sleep=no_sleep
    )
    assert report.confirmed is True
    assert report.suggested_bit_for_channel == [2, 3, 1, 0]
    assert report.suggested_black_is_raw_zero is False


def test_channel_calibration_rejects_unstable_baseline(clock, no_sleep):
    robot, _transport = make_robot([0b1111, 0b1011] * 30, clock=clock)
    report = calibrate_channels(
        robot, ask=lambda _p: "", samples_per_channel=5, interval_s=0.0, sleep=no_sleep
    )
    assert report.confirmed is False
    assert any("baseline was not stable" in note for note in report.notes)


def test_channel_calibration_rejects_multi_bit_changes(clock, no_sleep):
    # Covering "S1" also flips S2's bit, which must not be accepted.
    sequence = [0b1111] * 5 + [0b0011] * 5 + [0b1111] * 40
    robot, _transport = make_robot(sequence, clock=clock)
    report = calibrate_channels(
        robot, ask=lambda _p: "", samples_per_channel=5, interval_s=0.0, sleep=no_sleep
    )
    assert report.confirmed is False
    assert report.channel_bits["S1 (leftmost)"] is None


# -- crossing sweep ----------------------------------------------------------


class FakeSwitchResult:
    def __init__(self, completed: bool, reason: str = "") -> None:
        self.completed = completed
        self.reason = reason
        self.target_edge = EdgeState.BLACK_RIGHT


def test_crossing_sweep_records_success_rate_and_csv(tmp_path, clock):
    outcomes = {10.0: True, 15.0: True, 20.0: False}

    def attempt(angle: float):
        clock.advance(0.2)
        return FakeSwitchResult(outcomes[angle], "ok" if outcomes[angle] else "timeout")

    csv_path = tmp_path / "sweep.csv"
    sweep = CrossingSweep(attempt, csv_path=csv_path, clock=clock)
    report = sweep.run([10.0, 15.0, 20.0], attempts_per_angle=2)

    assert [entry.attempts for entry in report.entries] == [2, 2, 2]
    assert [entry.successes for entry in report.entries] == [2, 2, 0]
    assert report.entries[0].success_rate == 1.0
    assert report.best_angle_deg == 10.0
    assert report.confirmed is False

    with csv_path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    attempts = [row for row in rows if row["kind"] == "attempt"]
    summaries = [row for row in rows if row["kind"] == "summary"]
    assert len(attempts) == 6
    assert len(summaries) == 3
    assert attempts[0]["completed"] == "True"
    assert attempts[-1]["final_edge"] == "NOT_REACHED"
    assert "success_rate=0.000" in summaries[-1]["reason"]


def test_crossing_sweep_validates_attempts(clock):
    sweep = CrossingSweep(lambda _angle: FakeSwitchResult(True), clock=clock)
    with pytest.raises(ValueError):
        sweep.run([10.0], attempts_per_angle=0)


def test_crossing_sweep_recommends_nothing_when_every_attempt_fails(clock):
    sweep = CrossingSweep(
        lambda _angle: FakeSwitchResult(False, "timeout"), clock=clock
    )
    report = sweep.run([10.0, 20.0], attempts_per_angle=2)
    assert report.best_angle_deg is None
    assert report.confirmed is False
    assert all(entry.successes == 0 for entry in report.entries)
    assert all(entry.final_edge == "NOT_REACHED" for entry in report.entries)


# -- speed -------------------------------------------------------------------


def test_counts_to_mps_is_conservative():
    samples = [
        SpeedSample(counts=30, distance_m=1.0, duration_s=12.0),
        SpeedSample(counts=60, distance_m=1.0, duration_s=6.0),
    ]
    fastest = max(sample.mps_per_count for sample in samples)
    assert compute_counts_to_mps(samples, conservatism=1.0) == pytest.approx(
        fastest
    )
    assert compute_counts_to_mps(samples, conservatism=0.5) == pytest.approx(
        fastest / 0.5
    )
    assert compute_counts_to_mps([]) is None
    with pytest.raises(ValueError):
        compute_counts_to_mps(samples, conservatism=1.5)
    for bad in (
        SpeedSample(counts=0, distance_m=1.0, duration_s=1.0),
        SpeedSample(counts=1, distance_m=float("nan"), duration_s=1.0),
        SpeedSample(counts=1, distance_m=1.0, duration_s=-1.0),
    ):
        with pytest.raises(ValueError):
            compute_counts_to_mps([bad])


def test_derived_pwm_cap_never_exceeds_the_requested_speed():
    """Under the fastest measured ratio the derived counts stay within the limit."""
    samples = [
        SpeedSample(counts=30, distance_m=1.0, duration_s=10.0),
        SpeedSample(counts=60, distance_m=1.0, duration_s=5.0),
    ]
    factor = compute_counts_to_mps(samples, conservatism=0.8)
    fastest_mps_per_count = max(sample.mps_per_count for sample in samples)
    limit_mps = 0.15
    counts = int(limit_mps / factor)
    assert counts * fastest_mps_per_count <= limit_mps


def test_speed_report_and_instructions():
    report = SpeedCalibrationReport(
        samples=[SpeedSample(counts=40, distance_m=1.0, duration_s=10.0)]
    )
    payload = report.as_dict()
    # 40 counts covered 1 m in 10 s -> 0.0025 m/s per count, inflated by 0.8.
    assert payload["recommended_counts_to_mps"] == pytest.approx(0.0025 / 0.8)
    assert payload["fastest_speed_mps"] == pytest.approx(0.1)
    assert "counts_to_mps" in payload["instructions"]
    assert "FASTEST" in speed_calibration_instructions()
    assert "straf" in speed_calibration_instructions().lower()
    assert "Never enter a guessed factor" in speed_calibration_instructions()
