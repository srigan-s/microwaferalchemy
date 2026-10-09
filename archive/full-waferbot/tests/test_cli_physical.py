"""Physical-mode CLI behaviour with an injected transport and prompt.

No SSH and no hardware: the physical transport factory and the prompt are
injected, but everything else (map readiness, verified-config gate, manual
localization, stop-byte writing, obstacle monitor) runs the real code paths.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from waferbot import MockI2CTransport, RobotConfig, SafetyConfig
from waferbot.cli import main
from waferbot.process import ProcessGuard
from waferbot.sensing import EdgeState, edge_byte_for

STOP_PAYLOADS = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]


class UltrasonicCapableTransport(MockI2CTransport):
    """Mock transport that also answers ultrasonic distance registers."""

    def __init__(self, distance_mm: int = 9999, **kwargs) -> None:
        super().__init__(**kwargs)
        self.distance_mm = distance_mm

    def read_block(self, address: int, register: int, length: int) -> list[int]:
        if register == 0x1B:
            self.reads.append((address, register, length))
            return [(self.distance_mm >> 8) & 0xFF]
        if register == 0x1A:
            self.reads.append((address, register, length))
            return [self.distance_mm & 0xFF]
        return super().read_block(address, register, length)


def run(argv, capsys):
    code = main(argv)
    captured = capsys.readouterr()
    return code, captured.out, captured.err


def write_ready_map(tmp_path: Path) -> Path:
    data = json.loads(Path("maps/example_track.json").read_text(encoding="utf-8"))
    data["is_example"] = False
    data["physical_validated"] = True
    path = tmp_path / "track.json"
    path.write_text(json.dumps(data), encoding="utf-8")
    return path


def write_configs(tmp_path: Path, *, verified: bool = True) -> tuple[Path, Path]:
    base = RobotConfig().with_speed_limit(60)
    robot_config = RobotConfig(
        motor=type(base.motor)(**{**base.motor.__dict__, "verified": verified}),
        sensor=type(base.sensor)(**{**base.sensor.__dict__, "verified": verified}),
        safety=base.safety,
    ).validate()
    robot_path = tmp_path / "robot.json"
    robot_config.save_json(robot_path)
    nav = json.loads(Path("config/nav.example.json").read_text(encoding="utf-8"))
    nav["follow"]["controller_enabled"] = True
    nav["counts_to_mps"] = 0.01
    nav["counts_to_mps_note"] = "test fixture"
    nav_path = tmp_path / "nav.json"
    nav_path.write_text(json.dumps(nav), encoding="utf-8")
    return robot_path, nav_path


@pytest.fixture
def physical_env(tmp_path, monkeypatch):
    """Inject a physical transport and prompts; expose the created transports."""
    created: list[MockI2CTransport] = []
    prompts: list[str] = []

    def factory(config: RobotConfig):
        transport = UltrasonicCapableTransport(
            distance_mm=9999,
            line_sensor_byte=edge_byte_for(EdgeState.BLACK_LEFT, config.sensor),
        )
        created.append(transport)
        return transport

    def prompt(text: str) -> str:
        prompts.append(text)
        return "yes"

    monkeypatch.setattr("waferbot.cli.PHYSICAL_TRANSPORT_FACTORY", factory)
    monkeypatch.setattr("waferbot.cli._prompt", prompt)
    monkeypatch.setattr("waferbot.cli.INTERACTIVE_OVERRIDE", True)
    return {"created": created, "prompts": prompts, "runtime": tmp_path / "rt"}


def test_physical_execute_refuses_scripted_localizer(
    tmp_path, physical_env, capsys
) -> None:
    robot_config, nav_config = write_configs(tmp_path)
    track = write_ready_map(tmp_path)
    code, _out, err = run(
        [
            "execute", "--physical",
            "--start", "A+", "--goal", "B+",
            "--config", str(robot_config),
            "--nav-config", str(nav_config),
            "--map", str(track),
            "--runtime-dir", str(physical_env["runtime"]),
            "--localizer", "scripted",
        ],
        capsys,
    )
    assert code == 3
    assert "mock-only" in err
    assert physical_env["created"] == [], "no bus should be opened on refusal"


def test_physical_execute_uses_manual_localization(
    tmp_path, physical_env, capsys
) -> None:
    robot_config, nav_config = write_configs(tmp_path)
    track = write_ready_map(tmp_path)
    code, out, err = run(
        [
            "execute", "--physical",
            "--start", "A+", "--goal", "B+",
            "--config", str(robot_config),
            "--nav-config", str(nav_config),
            "--map", str(track),
            "--runtime-dir", str(physical_env["runtime"]),
            "--localizer", "manual",
            "--json",
        ],
        capsys,
    )
    assert code == 0, err
    payload = json.loads(out)
    assert payload["completed"] is True
    assert payload["final_localization"]["source"] == "manual"
    assert any(
        "is the robot at" in text.lower() for text in physical_env["prompts"]
    )


def test_physical_execution_requires_verified_configuration(
    tmp_path, physical_env, capsys
) -> None:
    robot_config, nav_config = write_configs(tmp_path, verified=False)
    track = write_ready_map(tmp_path)
    code, _out, err = run(
        [
            "execute", "--physical",
            "--start", "A+", "--goal", "B+",
            "--config", str(robot_config),
            "--nav-config", str(nav_config),
            "--map", str(track),
            "--runtime-dir", str(physical_env["runtime"]),
            "--localizer", "manual",
        ],
        capsys,
    )
    assert code == 3
    assert "not verified" in err


def test_physical_dry_run_never_opens_the_transport(
    tmp_path, physical_env, capsys
) -> None:
    robot_config, nav_config = write_configs(tmp_path)
    track = write_ready_map(tmp_path)
    code, out, _err = run(
        [
            "execute", "--physical", "--dry-run",
            "--start", "A+", "--goal", "D-", "--via", "B-",
            "--config", str(robot_config),
            "--nav-config", str(nav_config),
            "--map", str(track),
            "--runtime-dir", str(physical_env["runtime"]),
            "--json",
        ],
        capsys,
    )
    assert code == 0
    assert json.loads(out)["dry_run"] is True
    assert physical_env["created"] == [], "dry run must not open a bus"


def test_physical_stop_writes_stop_bytes(tmp_path, physical_env, capsys) -> None:
    runtime = tmp_path / "rt-stop"
    code, out, _err = run(
        ["stop", "--physical", "--runtime-dir", str(runtime), "--reason", "test halt"],
        capsys,
    )
    assert code == 0
    payload = json.loads(out)
    assert payload["stop_bytes_written"] == 4
    assert payload["bus_lock_acquired"] is True
    assert physical_env["created"][-1].payloads == STOP_PAYLOADS

    code, out, _err = run(["stop", "--runtime-dir", str(runtime), "--clear"], capsys)
    assert code == 0
    assert "cleared" in out


def test_clear_is_refused_while_a_session_owns_the_bus(tmp_path, capsys) -> None:
    runtime = tmp_path / "rt-owned"
    guard = ProcessGuard(runtime)
    guard.request_stop("earlier stop")
    guard.acquire_ownership("holder")
    try:
        code, _out, err = run(
            ["stop", "--runtime-dir", str(runtime), "--clear"], capsys
        )
        assert code == 3
        assert "still holds the bus" in err
        assert guard.stop_requested() is True
    finally:
        guard.release_ownership()

    code, _out, _err = run(["stop", "--runtime-dir", str(runtime), "--clear"], capsys)
    assert code == 0


def test_obstacle_monitor_runs_for_physical_sessions(tmp_path, capsys) -> None:
    """The configured monitor polls the real ranging path and can stop the robot."""
    import waferbot.cli as cli_module

    created: list[UltrasonicCapableTransport] = []

    def close_factory(config: RobotConfig):
        transport = UltrasonicCapableTransport(
            distance_mm=120,  # below the threshold
            line_sensor_byte=edge_byte_for(EdgeState.BLACK_LEFT, config.sensor),
        )
        created.append(transport)
        return transport

    robot_config, nav_config = write_configs(tmp_path)
    previous_factory = cli_module.PHYSICAL_TRANSPORT_FACTORY
    previous_prompt = cli_module._prompt
    previous_interactive = cli_module.INTERACTIVE_OVERRIDE
    cli_module.PHYSICAL_TRANSPORT_FACTORY = close_factory
    cli_module._prompt = lambda _text: "yes"
    cli_module.INTERACTIVE_OVERRIDE = True
    try:
        code, out, err = run(
            [
                "follow", "--physical",
                "--edge", "black-left",
                "--iterations", "3",
                "--obstacle-distance-mm", "200",
                "--config", str(robot_config),
                "--nav-config", str(nav_config),
                "--runtime-dir", str(tmp_path / "rt-obstacle"),
                "--ack-verified-config",
                "--json",
            ],
            capsys,
        )
    finally:
        cli_module.PHYSICAL_TRANSPORT_FACTORY = previous_factory
        cli_module._prompt = previous_prompt
        cli_module.INTERACTIVE_OVERRIDE = previous_interactive
    assert code == 3  # the obstacle stop aborts the follow
    assert created, "physical transport was not opened"
    # Ranging was enabled, then the obstacle stop wrote stop blocks.
    assert any(register == 0x07 for _a, register, _p in created[0].writes)
    assert created[0].payloads[-4:] == STOP_PAYLOADS
    assert not any(payload[2] for payload in created[0].payloads if len(payload) == 3)
    assert out == ""
    assert "OBSTACLE_DETECTED" in err or "emergency" in err.lower()


def test_sensors_work_without_a_map_from_another_directory(
    tmp_path, monkeypatch, capsys
) -> None:
    empty = tmp_path / "empty"
    empty.mkdir()
    monkeypatch.chdir(empty)
    code, out, err = run(["sensors", "--samples", "2"], capsys)
    assert code == 0, err
    assert "raw_byte" in out


def test_physical_sensors_is_truly_read_only(tmp_path, physical_env, capsys) -> None:
    """`sensors --physical` must not command motors, not even on exit."""
    robot_config, nav_config = write_configs(tmp_path)
    code, out, err = run(
        [
            "sensors", "--physical", "--samples", "2",
            "--config", str(robot_config),
            "--nav-config", str(nav_config),
            "--runtime-dir", str(physical_env["runtime"]),
        ],
        capsys,
    )
    assert code == 0, err
    assert "raw_byte" in out
    transport = physical_env["created"][-1]
    motor_writes = [
        payload for _address, register, payload in transport.writes if register == 0x01
    ]
    assert motor_writes == [], "read-only sensors must not write motor commands"
    assert transport.closed is True


def test_map_command_resolves_the_packaged_example(tmp_path, monkeypatch, capsys) -> None:
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)
    code, out, _err = run(["map", "--json"], capsys)
    assert code == 0
    payload = json.loads(out)
    assert payload["nodes"] == 21
    assert "example_track.json" in payload["path"]
