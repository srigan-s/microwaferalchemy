"""The standalone follow diagnostic script (model mode, no hardware)."""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SCRIPT = REPO / "scripts" / "test_line_follow.py"


def run_script(*args: str) -> subprocess.CompletedProcess:
    env = dict(os.environ)
    env["PYTHONDONTWRITEBYTECODE"] = "1"
    return subprocess.run(
        [sys.executable, str(SCRIPT), *args],
        capture_output=True,
        text=True,
        cwd=str(REPO),
        env=env,
    )


def test_model_mode_reports_both_edges():
    result = run_script("--edge", "both", "--json")
    assert result.returncode == 0, result.stderr
    reports = json.loads(result.stdout)
    assert [report["edge"] for report in reports] == ["BLACK_LEFT", "BLACK_RIGHT"]
    for report in reports:
        assert report["mode"] == "synthetic tape model"
        assert report["stop_reason"] == "MAX_ITERATIONS"
        assert report["converged"] is True
        assert report["tail_estimate_rms_pitches"] <= 0.5
        assert "not a hardware measurement" in report["note"]


def test_model_mode_reports_an_unfollowable_offset():
    result = run_script("--edge", "black-left", "--offset-mm", "500", "--json")
    assert result.returncode == 1
    report = json.loads(result.stdout)[0]
    assert report["stop_reason"] == "ACQUISITION_FAILED"
    assert report["travelled_m"] < 1e-6


def test_physical_mode_needs_confirmation():
    """`--physical` must never move without an explicit typed confirmation."""
    result = run_script("--physical", "--edge", "black-left", "--duration", "0.1")
    assert result.returncode != 0
    combined = result.stdout + result.stderr
    assert "not confirmed" in combined or "interactive confirmation" in combined


# Run the script's physical entry point with the real CLI session and an
# injected I2C transport. No robot or SSH connection is involved.
import importlib.util
import pytest
from waferbot import MockI2CTransport, RobotConfig
from waferbot.process import ProcessGuard
from waferbot.sensing import EdgeState, edge_byte_for
import waferbot.cli as cli


@pytest.fixture
def physical_script(tmp_path, monkeypatch):
    spec = importlib.util.spec_from_file_location("follow_diagnostic", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = RobotConfig()
    data = config.to_dict()
    data["motor"]["verified"] = True
    data["sensor"]["verified"] = True
    robot_path = tmp_path / "robot.json"
    robot_path.write_text(json.dumps(data))
    nav_data = json.loads((REPO / "config/nav.first-run.json").read_text())
    nav_data["follow"]["controller_enabled"] = True
    nav_path = tmp_path / "nav.json"
    nav_path.write_text(json.dumps(nav_data))
    runtime = tmp_path / "runtime"
    created = []

    def factory(config):
        transport = MockI2CTransport(line_sensor_byte=edge_byte_for(EdgeState.BLACK_LEFT))
        created.append(transport)
        return transport

    monkeypatch.setattr(cli, "PHYSICAL_TRANSPORT_FACTORY", factory)
    monkeypatch.setattr(cli, "INTERACTIVE_OVERRIDE", True)
    monkeypatch.setattr(cli, "_prompt", lambda _: "yes")
    argv = ["--physical", "--edge", "black-left", "--duration", "0.3",
            "--config", str(robot_path), "--nav-config", str(nav_path),
            "--runtime-dir", str(runtime), "--json"]
    return module, argv, runtime, created, robot_path


def test_physical_script_refuses_unverified_mapping_before_bus(physical_script, capsys):
    module, argv, runtime, created, config = physical_script
    data = json.loads(config.read_text())
    data["sensor"]["verified"] = False
    config.write_text(json.dumps(data))
    assert module.main(argv) == 3
    assert "not verified" in capsys.readouterr().err
    assert created == []


def test_physical_script_refuses_another_motion_owner(physical_script, capsys):
    module, argv, runtime, created, _ = physical_script
    owner = ProcessGuard(runtime)
    owner.acquire_ownership("another follower")
    try:
        assert module.main(argv) == 3
        assert created == []
    finally:
        owner.release_ownership()


def test_physical_script_refuses_existing_stop_latch(physical_script):
    module, argv, runtime, created, _ = physical_script
    ProcessGuard(runtime).request_stop("test")
    assert module.main(argv) == 3
    assert created == []


def test_physical_script_shared_stop_halts_and_prevents_new_motion(physical_script, monkeypatch):
    module, argv, runtime, created, _ = physical_script
    stopped_at = []

    class StopDuringFollow(MockI2CTransport):
        def read_block(self, address, register, length):
            if not stopped_at and any(p[2] for p in self.payloads):
                # This executes the same command a second SSH terminal runs.
                assert cli.main(["stop", "--physical", "--runtime-dir", str(runtime)]) == 0
                stopped_at.append(len(self.payloads))
            return super().read_block(address, register, length)

    def factory(config):
        kind = StopDuringFollow if not created else MockI2CTransport
        bus = kind(line_sensor_byte=edge_byte_for(EdgeState.BLACK_LEFT))
        created.append(bus)
        return bus

    monkeypatch.setattr(cli, "PHYSICAL_TRANSPORT_FACTORY", factory)
    assert module.main(argv) == 3
    bus = created[0]
    assert stopped_at, "the test must reach actual nonzero wheel commands"
    assert all(p[2] == 0 for p in bus.payloads[stopped_at[0]:])
    assert bus.closed
    assert ProcessGuard(runtime).stop_requested()
    # The completed session released ownership but did not clear the stop.
    guard = ProcessGuard(runtime)
    guard.acquire_ownership("check released")
    guard.release_ownership()


def test_physical_script_ctrl_c_stops_and_releases(physical_script, monkeypatch):
    module, argv, runtime, created, _ = physical_script

    class InterruptDuringFollow(MockI2CTransport):
        def read_block(self, address, register, length):
            if any(p[2] for p in self.payloads):
                raise KeyboardInterrupt
            return super().read_block(address, register, length)

    def factory(config):
        bus = InterruptDuringFollow(line_sensor_byte=edge_byte_for(EdgeState.BLACK_LEFT))
        created.append(bus)
        return bus

    monkeypatch.setattr(cli, "PHYSICAL_TRANSPORT_FACTORY", factory)
    assert module.main(argv) == 3
    assert created[0].payloads[-4:] == [(i, 0, 0) for i in range(4)]
    assert created[0].closed
    guard = ProcessGuard(runtime)
    guard.acquire_ownership("check released")
    guard.release_ownership()


def test_physical_script_never_reports_success_before_session_close(physical_script, monkeypatch, capsys):
    module, argv, runtime, created, _ = physical_script
    def factory(config):
        bus = MockI2CTransport(line_sensor_byte=edge_byte_for(EdgeState.BLACK_LEFT))
        # Initial and follower-final stop succeed; session-close stop fails.
        bus.fail_stops_after = 8
        created.append(bus)
        return bus
    monkeypatch.setattr(cli, "PHYSICAL_TRANSPORT_FACTORY", factory)
    assert module.main(argv) == 3
    captured = capsys.readouterr()
    assert '"stop_reason": "MAX_DURATION"' not in captured.out
    assert "stop" in captured.err
    assert created[0].closed


def test_physical_script_cannot_start_when_initial_stop_fails(physical_script, monkeypatch, capsys):
    module, argv, _, created, _ = physical_script
    def factory(config):
        bus = MockI2CTransport(line_sensor_byte=edge_byte_for(EdgeState.BLACK_LEFT))
        bus.fail_stops_after = 0
        created.append(bus)
        return bus
    monkeypatch.setattr(cli, "PHYSICAL_TRANSPORT_FACTORY", factory)
    assert module.main(argv) == 3
    assert not created[0].reads
    assert not any(p[2] for p in created[0].payloads)
    assert created[0].closed
