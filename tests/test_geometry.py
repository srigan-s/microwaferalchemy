"""Minimum crossing-angle geometry and its limits."""

from __future__ import annotations

import math

import pytest

from waferbot import ConfigError
from waferbot.geometry import (
    CAVEAT,
    CrossingInputs,
    evaluate_crossing,
    lateral_requirement_m,
    minimum_crossing_angle_deg,
    required_lateral_travel_m,
)
from waferbot.navconfig import GeometryConfig


def inputs(**overrides) -> CrossingInputs:
    payload = {
        "tape_width_m": 0.02,
        "sensor_spacing_m": 0.012,
        "sensor_detection_width_m": 0.004,
        "available_crossing_distance_m": 0.20,
        "safety_margin_m": 0.01,
        "robot_width_m": 0.15,
        "switching_speed_mps": 0.10,
        "sampling_rate_hz": 20.0,
        "lateral_clearance_m": 0.30,
        "required_confirmations": 3,
    }
    payload.update(overrides)
    return CrossingInputs(**payload)


def test_minimum_angle_matches_the_requested_formula():
    data = inputs()
    expected = math.degrees(math.atan((0.02 + 2 * 0.01) / 0.20))
    assert minimum_crossing_angle_deg(data) == pytest.approx(expected)
    assert minimum_crossing_angle_deg(data) == pytest.approx(11.3099, abs=1e-4)
    assert lateral_requirement_m(data) == pytest.approx(0.04)
    assert required_lateral_travel_m(data) == pytest.approx(0.04)


def test_evaluation_reports_every_constraint():
    evaluation = evaluate_crossing(inputs(), 20.0)
    assert evaluation.angle_ok is True
    assert evaluation.footprint_ok is True
    assert evaluation.clearance_ok is True
    assert evaluation.robot_clearance_ok is True
    assert evaluation.sampling_ok is True
    assert evaluation.lateral_clearance_ok is True
    assert evaluation.observability_ok is True
    assert evaluation.unambiguous_geometry is True
    assert evaluation.path_length_m == pytest.approx(0.04 / math.sin(math.radians(20)))
    assert evaluation.forward_advance_m == pytest.approx(0.04 / math.tan(math.radians(20)))
    assert evaluation.sample_distance_m == pytest.approx(0.005)
    assert evaluation.warnings[0] == CAVEAT


def test_angle_below_minimum_is_flagged():
    evaluation = evaluate_crossing(inputs(), 5.0)
    assert evaluation.angle_ok is False
    assert evaluation.unambiguous_geometry is False
    assert any("below the geometric minimum" in warning for warning in evaluation.warnings)


def test_tight_tape_cannot_be_resolved_by_the_sensor():
    # 0.005 m of lateral movement is smaller than one sensor pitch.
    evaluation = evaluate_crossing(
        inputs(tape_width_m=0.005, safety_margin_m=0.0), 20.0
    )
    assert evaluation.footprint_ok is False
    assert any("sensor pitch" in warning for warning in evaluation.warnings)


def test_limited_clearance_is_flagged():
    evaluation = evaluate_crossing(inputs(available_crossing_distance_m=0.03), 15.0)
    assert evaluation.clearance_ok is False
    assert any("forward travel" in warning for warning in evaluation.warnings)


def test_slow_sampling_relative_to_speed_is_flagged():
    evaluation = evaluate_crossing(
        inputs(switching_speed_mps=0.5, sampling_rate_hz=5.0), 20.0
    )
    assert evaluation.sampling_ok is False
    assert any("between samples" in warning for warning in evaluation.warnings)


def test_missing_lateral_clearance_is_flagged():
    evaluation = evaluate_crossing(inputs(lateral_clearance_m=0.0), 20.0)
    assert evaluation.lateral_clearance_ok is False
    assert evaluation.robot_clearance_ok is False
    assert any("lateral clearance" in warning for warning in evaluation.warnings)


def test_observability_uses_lateral_speed_rate_and_confirmations():
    # Fast lateral motion sampled slowly cannot give several confirmations
    # within the available lateral travel.
    evaluation = evaluate_crossing(
        inputs(
            switching_speed_mps=1.0,
            sampling_rate_hz=2.0,
            lateral_clearance_m=1.0,
            required_confirmations=5,
        ),
        30.0,
    )
    assert evaluation.observability_ok is False
    assert any("confirmations" in warning for warning in evaluation.warnings)


def test_inputs_can_be_built_from_the_commanded_values():
    geometry = GeometryConfig(
        tape_width_m=0.02,
        sensor_spacing_m=0.012,
        sensor_detection_width_m=0.004,
        available_crossing_distance_m=0.2,
        safety_margin_m=0.01,
        robot_width_m=0.15,
        switching_speed_mps=0.1,
        sampling_rate_hz=20.0,
        lateral_clearance_m=0.3,
        required_confirmations=3,
        measured=True,
    )
    built = CrossingInputs.from_measured_command(
        geometry, speed_counts=40, counts_to_mps=0.0025, rate_hz=20.0
    )
    assert built.switching_speed_mps == pytest.approx(0.1)
    assert built.sampling_rate_hz == 20.0
    with pytest.raises(ConfigError):
        CrossingInputs.from_measured_command(
            geometry, speed_counts=40, counts_to_mps=0.0, rate_hz=20.0
        )


@pytest.mark.parametrize(
    "bad",
    [
        {"tape_width_m": 0.0},
        {"tape_width_m": float("nan")},
        {"sensor_spacing_m": -1.0},
        {"sampling_rate_hz": 0.0},
        {"safety_margin_m": -0.001},
    ],
)
def test_invalid_inputs_are_rejected(bad):
    with pytest.raises(ConfigError):
        inputs(**bad)


def test_requested_angle_must_be_finite_and_inside_range():
    with pytest.raises(ConfigError):
        evaluate_crossing(inputs(), 0.0)
    with pytest.raises(ConfigError):
        evaluate_crossing(inputs(), 90.0)
    with pytest.raises(ConfigError):
        evaluate_crossing(inputs(), float("inf"))


def test_from_config_requires_measured_geometry():
    with pytest.raises(ConfigError):
        CrossingInputs.from_config(GeometryConfig())
    measured = GeometryConfig(
        tape_width_m=0.02,
        sensor_spacing_m=0.012,
        sensor_detection_width_m=0.004,
        available_crossing_distance_m=0.2,
        safety_margin_m=0.01,
        robot_width_m=0.15,
        switching_speed_mps=0.1,
        sampling_rate_hz=20.0,
        measured=True,
    )
    assert CrossingInputs.from_config(measured).tape_width_m == 0.02


def test_evaluation_serialises():
    payload = evaluate_crossing(inputs(), 20.0).as_dict()
    assert payload["unambiguous_geometry"] is True
    assert payload["requested_angle_deg"] == 20.0
    assert isinstance(payload["warnings"], list)
