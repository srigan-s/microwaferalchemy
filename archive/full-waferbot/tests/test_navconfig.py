"""Navigation configuration validation and persistence."""

from __future__ import annotations

import json

import pytest

from waferbot.errors import ConfigError
from waferbot.navconfig import (
    ExecutionConfig,
    FollowConfig,
    GeometryConfig,
    NavConfig,
    SwitchConfig,
)


def test_defaults_are_valid_and_uncalibrated():
    nav = NavConfig().validate()
    assert nav.follow.rate_hz == 100.0
    assert nav.follow.ir_polling_hz == 300.0
    assert nav.switch.mode == "lateral"
    assert nav.geometry.measured is False
    assert nav.has_speed_calibration is False
    assert nav.execution.allow_example_map is False


@pytest.mark.parametrize(
    "overrides",
    [
        {"rate_hz": 0.0},
        {"rate_hz": float("nan")},
        {"kp": -1.0},
        {"max_correction": -1},
        {"min_curve_speed_factor": 1.1},
        {"curve_slowdown_start": -0.1},
        {"correction_mode": "spin"},
        {"lateral_weight": 1.5},
        {"junction_speed_factor": 2.0},
        {"junction_channels": 5},
        {"edge_loss_samples": 0},
        {"stable_samples": 0},
        {"sensor_max_age_s": 0.0},
    ],
)
def test_follow_config_is_strict(overrides):
    with pytest.raises(ConfigError):
        FollowConfig(**overrides).validate()


@pytest.mark.parametrize(
    "overrides",
    [
        {"mode": "twist"},
        {"mode": "diagonal", "crossing_angle_deg": None},
        {"mode": "diagonal", "crossing_angle_deg": 0.0},
        {"mode": "diagonal", "crossing_angle_deg": 90.0},
        {"crossing_angle_deg": float("inf")},
        {"reduce_speed_factor": 1.5},
        {"confirm_samples": 0},
        {"switch_timeout_s": 0.0},
        {"authorization_max_age_s": -1.0},
    ],
)
def test_switch_config_is_strict(overrides):
    with pytest.raises(ConfigError):
        SwitchConfig(**overrides).validate()


def test_geometry_requires_measurements_to_be_positive():
    assert GeometryConfig().validate().measured is False
    with pytest.raises(ConfigError):
        GeometryConfig(measured=True).validate()
    with pytest.raises(ConfigError):
        GeometryConfig(
            tape_width_m=float("inf"),
            sensor_spacing_m=0.01,
            sensor_detection_width_m=0.004,
            available_crossing_distance_m=0.2,
            switching_speed_mps=0.1,
            sampling_rate_hz=20.0,
            measured=True,
        ).validate()
    good = GeometryConfig(
        tape_width_m=0.02,
        sensor_spacing_m=0.012,
        sensor_detection_width_m=0.004,
        available_crossing_distance_m=0.2,
        safety_margin_m=0.01,
        robot_width_m=0.15,
        switching_speed_mps=0.1,
        sampling_rate_hz=20.0,
        measured=True,
    ).validate()
    good.require_measurements("test")
    with pytest.raises(ConfigError):
        GeometryConfig().require_measurements("diagonal switch")


@pytest.mark.parametrize(
    "overrides",
    [
        {"max_edge_travel_s": 0.0},
        {"max_route_duration_s": float("nan")},
        {"turn_speed": 0},
        {"dock_speed": -1},
        {"allow_example_map": "yes"},
    ],
)
def test_execution_config_is_strict(overrides):
    with pytest.raises(ConfigError):
        ExecutionConfig(**overrides).validate()


def test_from_dict_rejects_unknown_sections_and_keys():
    with pytest.raises(ConfigError):
        NavConfig.from_dict({"planner": {}})
    with pytest.raises(ConfigError):
        NavConfig.from_dict({"follow": {"kpp": 1.0}})
    with pytest.raises(ConfigError):
        NavConfig.from_dict({"follow": []})


def test_counts_to_mps_must_be_positive_when_set():
    with pytest.raises(ConfigError):
        NavConfig.from_dict({"counts_to_mps": 0.0})
    with pytest.raises(ConfigError):
        NavConfig.from_dict({"counts_to_mps": -0.5})
    nav = NavConfig.from_dict({"counts_to_mps": 0.012, "counts_to_mps_note": "measured"})
    assert nav.has_speed_calibration is True
    assert nav.counts_to_mps == 0.012


def test_json_round_trip(tmp_path):
    nav = NavConfig().with_mode("diagonal", 18.0).with_follow(base_speed=25)
    path = tmp_path / "nav.json"
    nav.save_json(path)
    loaded = NavConfig.load_json(path)
    assert loaded.to_dict() == nav.to_dict()
    assert json.loads(path.read_text())["switch"]["mode"] == "diagonal"


def test_missing_or_malformed_config_is_an_error(tmp_path):
    with pytest.raises(ConfigError):
        NavConfig.load_json(tmp_path / "absent.json")
    broken = tmp_path / "broken.json"
    broken.write_text("{nope")
    with pytest.raises(ConfigError):
        NavConfig.load_json(broken)


def test_example_config_in_repository_is_valid():
    nav = NavConfig.load_json("config/nav.example.json")
    assert nav.map_path == "maps/example_track.json"
    assert nav.geometry.measured is False


def test_windy_first_run_profile_is_bounded_and_disables_recovery():
    nav = NavConfig.load_json("config/nav.windy-first-run.json")
    assert nav.follow.base_speed == 5
    assert nav.follow.max_correction == 5
    assert nav.follow.kp_pwm_per_mm == pytest.approx(5 / 6.5, rel=1e-5)
    assert nav.follow.max_correction_slew_pwm_per_s == 60
    assert nav.follow.ir_polling_hz == 300
    assert nav.follow.rate_hz == 100
    assert nav.follow.controller_enabled is False
    assert nav.follow.min_curve_speed_factor == 0.2
    assert nav.follow.curve_slowdown_start == 0.0
    assert nav.follow.recovery_enabled is False


@pytest.mark.parametrize(
    "name", ["nav.first-run.json", "nav.example.json", "nav.tuned.json"]
)
def test_shipped_full_navigation_profiles_respect_five_count_limit(name):
    nav = NavConfig.load_json(f"config/{name}")
    assert max(
        nav.follow.base_speed,
        nav.follow.recovery_speed,
        nav.switch.speed,
        nav.switch.approach_speed,
        nav.execution.turn_speed,
        nav.execution.dock_speed,
    ) <= 5
