"""Straight angle-sweep geometry, timing evidence, and bounded motor behavior."""

from __future__ import annotations

import csv
from dataclasses import replace

import pytest

from waferbot.cli import _crossing_angles, main
from waferbot.config import RobotConfig
from waferbot.edge_angle_experiment import (
    minimum_confirmed_angle,
    run_straight_crossing,
    straight_wheels,
    summarize_angles,
)
from waferbot.hardware.mock import MockI2CTransport
from waferbot.robot import Robot
from waferbot.sensing.edge import EdgeState, edge_byte_for


def read_csv(path):
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def test_ten_degree_sweep_and_quantized_constant_wheel_vectors():
    assert _crossing_angles("10:80:10") == list(range(10, 90, 10))
    assert _crossing_angles("10,25,40") == [10, 25, 40]
    with pytest.raises(ValueError):
        _crossing_angles("0:90:10")
    for requested in range(10, 90, 10):
        wheels, realized = straight_wheels(requested, 5)
        assert wheels[0] == wheels[3]
        assert wheels[1] == wheels[2]
        assert max(map(abs, wheels)) <= 5
        assert sum(wheels) > 0  # forward component
        assert wheels[1] > wheels[0]  # leftward component
        assert abs(realized - requested) <= 6
    assert straight_wheels(40, 5)[0] == straight_wheels(50, 5)[0]


def test_crossing_logs_every_edge_change_and_confirms_target(tmp_path):
    config = RobotConfig().with_speed_limit(5)
    source = edge_byte_for(EdgeState.BLACK_LEFT, config.sensor)
    middle = edge_byte_for(EdgeState.BOTH_BLACK, config.sensor)
    target = edge_byte_for(EdgeState.BLACK_RIGHT, config.sensor)
    transport = MockI2CTransport(
        line_sensor_sequence=[source] * 8 + [middle] * 3 + [target] * 30
    )
    robot = Robot(transport, config, watchdog=False)
    robot.arm()
    trace = tmp_path / "crossing.csv"
    result = run_straight_crossing(
        robot, trace, angle_deg=30, speed=5, duration_s=0.3,
        sample_rate_hz=100, confirm_ms=20,
    )
    assert result["stop_reason"] == "TARGET_CONFIRMED"
    assert result["source_acquired"] is True
    assert result["target_confirmed"] is True
    assert result["switch_time_ms"] > 0
    assert result["confirmation_time_ms"] >= result["switch_time_ms"] + 20
    assert [(event["from"], event["to"]) for event in result["edge_changes"]] == [
        ("BLACK_LEFT", "BOTH_BLACK"), ("BOTH_BLACK", "BLACK_RIGHT")
    ]
    rows = read_csv(trace)
    assert [row["phase"] for row in rows[:3]] == ["baseline"] * 3
    assert any(row["edge_changed"] == "1" for row in rows)
    assert all(int(payload[2]) <= 5 for _, _, payload in transport.writes)
    moving_commands = [payload for _, _, payload in transport.writes if payload[2] > 0]
    assert moving_commands
    assert [payload for _, _, payload in transport.writes[-4:]] == [
        (motor_id, 0, 0) for motor_id in range(4)
    ]
    robot.close()


def test_wrong_source_never_moves_and_minimum_requires_all_repeats(tmp_path):
    config = RobotConfig().with_speed_limit(5)
    transport = MockI2CTransport(
        line_sensor_byte=edge_byte_for(EdgeState.BOTH_WHITE, config.sensor)
    )
    robot = Robot(transport, config, watchdog=False)
    robot.arm()
    result = run_straight_crossing(
        robot, tmp_path / "no-source.csv", angle_deg=20, speed=5,
        duration_s=0.05, sample_rate_hz=100,
    )
    assert result["stop_reason"] == "SOURCE_NOT_ACQUIRED"
    assert result["moving_samples"] == 0
    assert all(payload[2] == 0 for _, _, payload in transport.writes)
    robot.close()
    evidence = [
        {"angle_deg": 10, "target_confirmed": True},
        {"angle_deg": 10, "target_confirmed": False},
        {"angle_deg": 20, "target_confirmed": True},
        {"angle_deg": 20, "target_confirmed": True},
    ]
    assert minimum_confirmed_angle(evidence, 2) == 20


def test_summary_flags_duplicate_pwm_angles():
    evidence = [
        {"angle_deg": angle, "commanded_angle_deg": straight_wheels(angle, 5)[1],
         "speed_pwm": 5, "target_confirmed": True, "switch_time_ms": 40,
         "observed_rate_hz": 300}
        for angle in (40.0, 50.0)
    ]
    summary = summarize_angles(evidence, [40.0, 50.0], 1)
    assert summary[0]["duplicate_command_of_deg"] is None
    assert summary[1]["duplicate_command_of_deg"] == 40.0
    assert summary[1]["median_switch_time_ms"] == 40


def test_physical_sweep_prompts_before_fixed_vector_and_writes_summary(
    tmp_path, monkeypatch, capsys
):
    import waferbot.cli as cli

    config = RobotConfig().with_speed_limit(5)
    config = replace(
        config,
        motor=replace(config.motor, verified=True),
        sensor=replace(config.sensor, verified=True),
    )
    config_path = tmp_path / "robot.json"
    config.save_json(config_path)
    source = edge_byte_for(EdgeState.BLACK_LEFT, config.sensor)
    target = edge_byte_for(EdgeState.BLACK_RIGHT, config.sensor)
    created = []
    prompts = []

    def factory(_config):
        transport = MockI2CTransport(
            line_sensor_sequence=[source] * 8 + [target] * 20
        )
        created.append(transport)
        return transport

    def prompt(message):
        prompts.append(message)
        return "yes" if len(prompts) == 1 else "ready"

    monkeypatch.setattr(cli, "PHYSICAL_TRANSPORT_FACTORY", factory)
    monkeypatch.setattr(cli, "INTERACTIVE_OVERRIDE", True)
    monkeypatch.setattr(cli, "_prompt", prompt)
    output_dir = tmp_path / "sweep"
    code = main([
        "edge-angle-sweep", "--physical", "--angles", "30",
        "--attempts", "1", "--speed-pwm", "5", "--sample-rate-hz", "100",
        "--duration", "0.2", "--confirm-ms", "20",
        "--config", str(config_path), "--runtime-dir", str(tmp_path / "rt"),
        "--output-dir", str(output_dir),
    ])
    assert code == 0, capsys.readouterr().err
    assert len(prompts) == 2
    assert "WHEELS STOPPED" in prompts[1]
    summary = read_csv(output_dir / "summary.csv")
    assert len(summary) == 1
    assert summary[0]["target_confirmed"] == "True"
    assert float(summary[0]["switch_time_ms"]) > 0
    assert created[0].closed
    assert all(payload[2] <= 5 for _, _, payload in created[0].writes)
    assert [payload for _, _, payload in created[0].writes[-4:]] == [
        (motor_id, 0, 0) for motor_id in range(4)
    ]
