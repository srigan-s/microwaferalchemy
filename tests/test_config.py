"""Configuration validation and JSON round-trip."""

from __future__ import annotations

import json

import pytest

from waferbot import ConfigError, RobotConfig, SensorConfig


def test_defaults_are_valid():
    config = RobotConfig().validate()
    assert config.motor.address == 0x2B
    assert config.motor.bus == 1
    assert config.sensor.register == 0x0A
    assert config.sensor.bit_for_channel == (2, 3, 1, 0)
    assert config.sensor.black_is_raw_zero is True


def test_unknown_sections_are_rejected():
    with pytest.raises(ConfigError):
        RobotConfig.from_dict({"motor": {}, "planner": {}})


def test_unknown_keys_are_rejected():
    with pytest.raises(ConfigError):
        RobotConfig.from_dict({"motor": {"max_speed": 50, "speed": 60}})


def test_bad_speed_limit_is_rejected():
    with pytest.raises(ConfigError):
        RobotConfig.from_dict({"motor": {"max_speed": 0}})
    with pytest.raises(ConfigError):
        RobotConfig.from_dict({"motor": {"max_speed": 300}})


def test_bit_mapping_must_be_a_permutation():
    with pytest.raises(ConfigError):
        SensorConfig(bit_for_channel=(2, 2, 1, 0)).validate()


def test_timeouts_must_be_finite_and_positive():
    with pytest.raises(ConfigError):
        RobotConfig.from_dict({"safety": {"motion_timeout_s": float("inf")}})
    with pytest.raises(ConfigError):
        RobotConfig.from_dict({"safety": {"motion_timeout_s": -1}})


def test_json_round_trip(tmp_path):
    config = RobotConfig().with_speed_limit(42)
    path = tmp_path / "robot.json"
    config.save_json(path)
    loaded = RobotConfig.load_json(path)
    assert loaded.to_dict() == config.to_dict()
    assert json.loads(path.read_text())["motor"]["max_speed"] == 42


def test_missing_config_file_is_a_config_error(tmp_path):
    with pytest.raises(ConfigError):
        RobotConfig.load_json(tmp_path / "absent.json")


def test_malformed_config_file_is_a_config_error(tmp_path):
    path = tmp_path / "broken.json"
    path.write_text("{not json")
    with pytest.raises(ConfigError):
        RobotConfig.load_json(path)


def test_unverified_hardware_assumptions_are_reported():
    from waferbot import SafetyError
    from waferbot.config import MotorConfig, SensorConfig

    config = RobotConfig()
    pending = config.unverified_components()
    assert len(pending) == 2
    assert any("wheel order" in item for item in pending)
    assert any("channel order" in item for item in pending)
    with pytest.raises(SafetyError):
        config.require_verified_hardware()

    verified = RobotConfig(
        motor=MotorConfig(verified=True), sensor=SensorConfig(verified=True)
    ).validate()
    assert verified.unverified_components() == []
    verified.require_verified_hardware()


def test_verified_flags_must_be_boolean():
    from waferbot.config import MotorConfig, SensorConfig

    with pytest.raises(ConfigError):
        MotorConfig(verified="yes").validate()  # type: ignore[arg-type]
    with pytest.raises(ConfigError):
        SensorConfig(verified=1).validate()  # type: ignore[arg-type]


def test_inverted_wheels_helper():
    config = RobotConfig().with_inverted_wheels([1, 3])
    assert config.motor.invert == (False, True, False, True)
    with pytest.raises(ConfigError):
        RobotConfig().with_inverted_wheels([4])
