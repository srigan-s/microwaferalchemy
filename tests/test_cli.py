"""CLI behaviour in mock mode, refusal paths, and process-safe stop handling."""

from __future__ import annotations

import json
import re

import pytest

from waferbot.cli import main
from waferbot.telemetry import read_rows


def run(argv, capsys):
    code = main(argv)
    captured = capsys.readouterr()
    return code, captured.out, captured.err


def test_sensors_mock_prints_raw_and_normalised(capsys):
    code, out, _err = run(["sensors", "--samples", "2"], capsys)
    assert code == 0
    assert "raw_byte" in out
    assert "0x07" in out


def test_sensors_json_output(capsys):
    code, out, _err = run(["sensors", "--samples", "2", "--json"], capsys)
    assert code == 0
    payload = json.loads(out)
    assert len(payload["samples"]) == 2
    assert payload["samples"][0]["raw_byte"] == 0x07
    assert payload["samples"][0]["normalized"] == [0, 1, 0, 0]


def test_map_command_reports_readiness(capsys):
    code, out, _err = run(["map"], capsys)
    assert code == 0
    payload = json.loads(out)
    assert payload["is_example"] is True
    assert payload["physical_ready"] is False
    assert payload["nodes"] == 21

    code, _out, _err = run(["map", "--require-physical"], capsys)
    assert code == 3


def test_plan_command(capsys):
    code, out, _err = run(["plan", "--start", "A+", "--goal", "D-", "--via", "B-"], capsys)
    assert code == 0
    payload = json.loads(out)
    assert payload["nodes"][0] == "A+"
    assert payload["nodes"][-1] == "D-"
    assert "B-" in payload["nodes"]
    assert any(step["action"] == "SWITCH_EDGE" for step in payload["steps"])

    code, out, _err = run(
        ["plan", "--start", "A+", "--goal", "D-", "--via", "B-", "--algorithm", "astar"],
        capsys,
    )
    assert code == 0


def test_plan_unknown_node_is_refused(capsys):
    code, _out, err = run(["plan", "--start", "nope", "--goal", "D-"], capsys)
    assert code == 3
    assert "unknown" in err


def test_execute_dry_run_touches_nothing(capsys):
    code, out, _err = run(
        ["execute", "--start", "A+", "--goal", "D-", "--via", "B-", "--dry-run"], capsys
    )
    assert code == 0
    payload = json.loads(out)
    assert payload["dry_run"] is True
    assert payload["completed"] is True


def test_execute_mock_end_to_end(capsys):
    code, out, _err = run(
        ["execute", "--start", "A+", "--goal", "D-", "--via", "B-"], capsys
    )
    assert code == 0
    payload = json.loads(out)
    assert payload["completed"] is True
    assert payload["final_localization"]["node_id"] == "D-"
    assert [step["action"] for step in payload["steps"]].count("SWITCH_EDGE") >= 1


def test_execute_writes_telemetry_csv(tmp_path, capsys):
    csv_path = tmp_path / "run.csv"
    code, _out, _err = run(
        [
            "execute",
            "--start",
            "A+",
            "--goal",
            "D-",
            "--via",
            "B-",
            "--log-csv",
            str(csv_path),
        ],
        capsys,
    )
    assert code == 0
    rows = read_rows(csv_path)
    events = {row["event"] for row in rows}
    assert {"sensor", "motor"} <= events
    assert any(row["event"] == "localization" for row in rows)


def test_follow_mock_command(capsys):
    code, out, _err = run(
        ["follow", "--edge", "black-left", "--iterations", "3"], capsys
    )
    assert code == 0
    payload = json.loads(out)
    assert payload["target_edge"] == "BLACK_LEFT"
    assert payload["samples"] == 3
    assert payload["stop_reason"] == "MAX_ITERATIONS"


def test_switch_requires_authorization(capsys):
    code, out, _err = run(["switch", "--to-edge", "black-right"], capsys)
    assert code == 3
    assert "authorization" in out


def test_switch_mock_command(capsys):
    code, out, _err = run(
        [
            "switch",
            "--from-edge",
            "black-left",
            "--to-edge",
            "black-right",
            "--location",
            "B2",
            "--route-id",
            "route-test",
            "--authorize",
        ],
        capsys,
    )
    assert code == 0
    payload = json.loads(out)
    assert payload["completed"] is True
    assert payload["target_edge"] == "BLACK_RIGHT"


def test_motor_test_mock_marks_mapping_uncertain(capsys):
    code, out, _err = run(["motor-test", "--mock"], capsys)
    assert code == 0
    payload = json.loads(out)
    assert payload["confirmed"] is False
    assert payload["uncertain_until_confirmed"] is True
    assert len(payload["probes"]) == 4


def test_calibrate_speed_command(capsys):
    code, out, _err = run(
        [
            "calibrate",
            "speed",
            "--counts",
            "40",
            "--distance-m",
            "1.0",
            "--duration-s",
            "10.0",
        ],
        capsys,
    )
    assert code == 0
    assert "recommended_counts_to_mps" in out
    # 1 m in 10 s at 40 counts -> 0.0025 m/s per count, inflated by 0.8.
    match = re.search(r'"recommended_counts_to_mps":\s*([0-9.eE+-]+)', out)
    assert match is not None
    assert float(match.group(1)) == pytest.approx(0.0025 / 0.8)
    assert "FASTEST" in out


def test_calibrate_speed_without_numbers_is_usage(capsys):
    code, out, _err = run(["calibrate", "speed"], capsys)
    assert code == 2
    assert "Speed calibration" in out

    code, out, _err = run(["calibrate", "speed", "--instructions"], capsys)
    assert code == 0
    assert "counts_to_mps" in out


def test_calibrate_crossing_refuses_without_measurements(capsys):
    code, out, _err = run(["calibrate", "crossing", "--mock"], capsys)
    assert code == 3
    assert "measured geometry" in out


def test_calibrate_crossing_mock_sweep(tmp_path, capsys):
    nav = json.loads(
        open("config/nav.example.json", encoding="utf-8").read()
    )
    nav["geometry"].update(
        {
            "tape_width_m": 0.02,
            "sensor_spacing_m": 0.012,
            "sensor_detection_width_m": 0.004,
            "available_crossing_distance_m": 0.2,
            "safety_margin_m": 0.01,
            "robot_width_m": 0.15,
            "switching_speed_mps": 0.1,
            "sampling_rate_hz": 20.0,
            "lateral_clearance_m": 0.3,
            "required_confirmations": 3,
            "measured": True,
        }
    )
    nav_path = tmp_path / "nav.json"
    nav_path.write_text(json.dumps(nav), encoding="utf-8")
    csv_path = tmp_path / "sweep.csv"

    code, out, _err = run(
        [
            "calibrate",
            "crossing",
            "--mock",
            "--nav-config",
            str(nav_path),
            "--angles",
            "15",
            "--attempts",
            "1",
            "--csv",
            str(csv_path),
        ],
        capsys,
    )
    assert code == 0
    payload = json.loads(out)
    assert payload["entries"][0]["success_rate"] == 1.0
    assert csv_path.exists()


def test_physical_command_needs_an_interactive_confirmation(capsys):
    code, _out, err = run(
        ["follow", "--physical", "--edge", "black-left", "--duration", "0.1"], capsys
    )
    assert code == 3
    assert "interactive confirmation" in err


def test_stop_latch_blocks_motion_until_cleared(tmp_path, capsys):
    runtime = str(tmp_path / "runtime")
    code, out, _err = run(["stop", "--runtime-dir", runtime, "--reason", "test"], capsys)
    assert code == 0
    assert json.loads(out)["stop_requested"] is True

    code, out, _err = run(["stop", "--runtime-dir", runtime, "--status"], capsys)
    assert code == 0
    status = json.loads(out)
    assert status["stop_requested"] is True
    assert status["session"] is None

    code, _out, err = run(
        ["follow", "--mock", "--runtime-dir", runtime, "--edge", "black-left"],
        capsys,
    )
    assert code == 3
    assert "stop was requested" in err

    code, out, _err = run(["stop", "--runtime-dir", runtime, "--clear"], capsys)
    assert code == 0
    assert "cleared" in out

    code, _out, _err = run(
        [
            "follow",
            "--mock",
            "--runtime-dir",
            runtime,
            "--edge",
            "black-left",
            "--iterations",
            "2",
        ],
        capsys,
    )
    assert code == 0


def test_invalid_config_path_is_refused(tmp_path, capsys):
    code, _out, err = run(
        ["sensors", "--config", str(tmp_path / "missing.json")], capsys
    )
    assert code == 3
    assert "not found" in err


def test_unknown_map_is_refused(tmp_path, capsys):
    code, _out, err = run(["map", "--map", str(tmp_path / "missing.json")], capsys)
    assert code == 3
    assert "not found" in err
