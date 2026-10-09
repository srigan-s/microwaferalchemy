"""Timing, CSV, and stop behavior for the bounded IR strafe experiment."""

from __future__ import annotations

import csv
import json
from dataclasses import replace

import pytest

from waferbot.cli import main
from waferbot.config import RobotConfig
from waferbot.hardware.mock import MockI2CTransport
from waferbot.ir_experiment import run_ir_strafe
from waferbot.robot import Robot


def rows(path):
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def test_mock_cli_records_baseline_and_timed_moving_reads(tmp_path, capsys):
    path = tmp_path / "ir.csv"
    code = main([
        "ir-strafe", "--speed-pwm", "5", "--sample-rate-hz", "100",
        "--duration", "0.06", "--output-csv", str(path), "--json",
    ])
    output = json.loads(capsys.readouterr().out)
    data = rows(path)
    assert code == 0
    assert output["moving_samples"] >= 3
    assert output["observed_rate_hz"] is not None
    assert data[0]["phase"] == "baseline"
    assert all(row["phase"] == "moving" for row in data[1:])
    assert len(data) == output["moving_samples"] + 1
    assert data[1]["raw_byte_hex"].startswith("0x")
    assert data[1]["black_s1"] in {"0", "1"}
    assert all(int(data[index]["read_end_ns"]) < int(data[index + 1]["read_end_ns"])
               for index in range(1, len(data) - 1))


def test_external_stop_ends_strafe_and_stops_motors(tmp_path):
    transport = MockI2CTransport()
    robot = Robot(transport, RobotConfig().with_speed_limit(5), watchdog=False)
    robot.arm()
    result = run_ir_strafe(
        robot, tmp_path / "stopped.csv", speed=5, rate_hz=100,
        duration_s=1.0, stop_requested=lambda: transport.read_attempts >= 3,
    )
    assert result["stop_reason"] == "STOP_REQUESTED"
    assert 1 <= result["moving_samples"] <= 2
    assert [payload for _, _, payload in transport.writes[-4:]] == [
        (motor_id, 0, 0) for motor_id in range(4)
    ]
    assert all(payload[2] <= 5 for _, _, payload in transport.writes if len(payload) == 3)
    robot.close()


def test_sensor_failure_still_stops(tmp_path):
    transport = MockI2CTransport()

    def fail_on_third_read():
        if transport.read_attempts == 3:
            transport.read_error = OSError("injected")

    transport.before_read = fail_on_third_read
    robot = Robot(transport, RobotConfig().with_speed_limit(5), watchdog=False)
    robot.arm()
    with pytest.raises(Exception, match="injected"):
        run_ir_strafe(robot, tmp_path / "failure.csv", speed=5, rate_hz=100, duration_s=1.0)
    assert [payload for _, _, payload in transport.writes[-4:]] == [
        (motor_id, 0, 0) for motor_id in range(4)
    ]
    robot.close()


def test_invalid_knobs_refuse_before_motion(tmp_path, capsys):
    path = tmp_path / "invalid.csv"
    code = main([
        "ir-strafe", "--speed-pwm", "6", "--config", "config/robot.first-run.json",
        "--output-csv", str(path),
    ])
    captured = capsys.readouterr()
    assert code == 2
    assert "1..5" in captured.err
    assert not path.exists()


def test_physical_cli_uses_shared_motion_session_and_five_count_cap(
    tmp_path, monkeypatch, capsys
):
    import waferbot.cli as cli

    created = []
    prompts = []

    def factory(_config):
        transport = MockI2CTransport()
        created.append(transport)
        return transport

    def prompt(message):
        prompts.append(message)
        return "yes"

    monkeypatch.setattr(cli, "PHYSICAL_TRANSPORT_FACTORY", factory)
    monkeypatch.setattr(cli, "INTERACTIVE_OVERRIDE", True)
    monkeypatch.setattr(cli, "_prompt", prompt)
    config = RobotConfig().with_speed_limit(5)
    config = replace(
        config,
        motor=replace(config.motor, verified=True),
        sensor=replace(config.sensor, verified=True),
    )
    config_path = tmp_path / "robot.json"
    config.save_json(config_path)
    path = tmp_path / "physical.csv"
    code = main([
        "ir-strafe", "--physical", "--speed-pwm", "5",
        "--sample-rate-hz", "100", "--duration", "0.04",
        "--config", str(config_path), "--runtime-dir", str(tmp_path / "rt"),
        "--output-csv", str(path),
    ])
    assert code == 0, capsys.readouterr().err
    assert len(prompts) == 1
    assert "REAL hardware" in prompts[0]
    assert rows(path)[0]["phase"] == "baseline"
    assert len(created) == 1
    assert created[0].closed
    assert any(payload[2] == 5 for _, _, payload in created[0].writes)
    assert all(payload[2] <= 5 for _, _, payload in created[0].writes)
    assert [payload for _, _, payload in created[0].writes[-4:]] == [
        (motor_id, 0, 0) for motor_id in range(4)
    ]
