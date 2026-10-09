"""The visible project follows tape with the safe hardware facade."""

import csv
from dataclasses import replace

import pytest

from waferbot.config import RobotConfig
from waferbot.cli import main as cli_main
from waferbot.errors import SensorError
from waferbot.hardware.mock import MockI2CTransport
from waferbot.pidconfig import PIDConfig
from waferbot.process import ProcessGuard
from waferbot.robot import Robot
from waferbot.sensing.edge import EdgeState
from waferbot.sensing.follower import PIDFollower, wheel_command
from waferbot.sensing.poller import PollSnapshot
from waferbot.sensing.position import estimate_position


class FakeClock:
    def __init__(self):
        self.value = 1.0

    def __call__(self):
        self.value += 0.001
        return self.value

    def sleep(self, delay):
        self.value += max(delay, 0.001)


def robot_for(sequence):
    clock = FakeClock()
    transport = MockI2CTransport(line_sensor_sequence=list(sequence))
    robot = Robot(transport, RobotConfig().with_speed_limit(5), clock=clock, watchdog=False)
    robot.arm()
    return robot, transport, clock


def test_position_and_stateless_lateral_mixer():
    centre = estimate_position((1, 1, 0, 0), EdgeState.BLACK_LEFT, 1)
    off_centre = estimate_position((1, 0, 0, 0), EdgeState.BLACK_LEFT, 2)
    assert centre.robot_mm == 0
    assert off_centre.robot_mm == 3.25
    assert [wheel_command(0, 2.25, limit=5) for _ in range(4)] == [
        (2, -2, -2, 2)
    ] * 4


def test_disabled_config_cannot_move():
    robot, transport, clock = robot_for([0x03] * 20)
    follower = PIDFollower(robot, PIDConfig(), clock=clock, sleep=clock.sleep, use_async_poller=False)
    with pytest.raises(ValueError, match="disabled"):
        follower.run(EdgeState.BLACK_LEFT)
    assert not transport.motor_payloads
    robot.close()


def test_centred_follow_sends_forward_integer_pwm_and_stops(tmp_path):
    robot, transport, clock = robot_for([0x03] * 50)
    cfg = replace(PIDConfig(), enabled=True, acquisition_samples=2)
    path = tmp_path / "control.csv"
    result = PIDFollower(robot, cfg, clock=clock, sleep=clock.sleep, use_async_poller=False).run(
        EdgeState.BLACK_LEFT, duration_s=0.2, log_csv=path
    )
    assert result.stop_reason == "MAX_DURATION" and result.samples > 0
    assert path.exists()
    nonzero = [payload for payload in transport.motor_payloads if payload[2] > 0]
    assert nonzero and all(payload[2] == 4 for payload in nonzero)
    assert transport.motor_payloads[-4:] == [(i, 0, 0) for i in range(4)]
    robot.close()


def test_off_centre_reading_drives_pid_lateral_correction(tmp_path):
    robot, transport, clock = robot_for([0x0B] * 50)  # normalized 1000
    cfg = replace(
        PIDConfig(), enabled=True, acquisition_samples=2,
        kp=1.0, kd=0.0, max_slew_pwm_per_s=1000,
    )
    path = tmp_path / "control.csv"
    result = PIDFollower(robot, cfg, clock=clock, sleep=clock.sleep, use_async_poller=False).run(
        EdgeState.BLACK_LEFT, duration_s=0.2, log_csv=path
    )
    assert result.stop_reason == "MAX_DURATION" and result.samples > 0
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert all(float(row["robot_position_mm"]) == 3.25 for row in rows)
    assert all(float(row["error_mm"]) == -3.25 for row in rows)
    assert any(int(row["front_left_pwm"]) != int(row["rear_left_pwm"]) for row in rows)
    assert all(abs(int(row[key])) <= 5 for row in rows for key in (
        "front_left_pwm", "rear_left_pwm", "front_right_pwm", "rear_right_pwm"
    ))
    assert transport.motor_payloads[-4:] == [(i, 0, 0) for i in range(4)]
    robot.close()


def test_loss_and_stop_latch_never_leave_motors_running():
    robot, transport, clock = robot_for([0x03] * 3 + [0xFF] * 30)
    cfg = replace(PIDConfig(), enabled=True, acquisition_samples=2, edge_loss_samples=2)
    result = PIDFollower(robot, cfg, clock=clock, sleep=clock.sleep, use_async_poller=False).run(
        EdgeState.BLACK_LEFT, duration_s=0.4
    )
    assert result.stop_reason == "LINE_LOST"
    assert transport.motor_payloads[-4:] == [(i, 0, 0) for i in range(4)]
    robot.close()

    robot, transport, clock = robot_for([0x03] * 30)
    result = PIDFollower(
        robot, cfg, clock=clock, sleep=clock.sleep,
        stop_requested=lambda: True, use_async_poller=False,
    ).run(EdgeState.BLACK_LEFT, duration_s=0.4)
    assert result.stop_reason == "STOP_REQUESTED"
    assert not any(payload[2] > 0 for payload in transport.motor_payloads)
    robot.close()


def test_pid_config_keeps_five_count_cap():
    with pytest.raises(ValueError, match="max_pwm"):
        replace(PIDConfig(), max_pwm=6).validate()
    assert PIDConfig.load_json("config/pid.first-run.json").enabled is False


def test_physical_style_poller_outpaces_control_loop():
    transport = MockI2CTransport(line_sensor_byte=0x03)
    robot = Robot(transport, RobotConfig().with_speed_limit(5), watchdog=False)
    robot.arm()
    cfg = replace(PIDConfig(), enabled=True, acquisition_samples=1)
    result = PIDFollower(robot, cfg).run(EdgeState.BLACK_LEFT, duration_s=0.2)
    assert result.stop_reason == "MAX_DURATION"
    assert result.samples > 0
    assert transport.read_attempts > result.samples
    assert transport.motor_payloads[-4:] == [(i, 0, 0) for i in range(4)]
    robot.close()


def test_no_fresh_poller_sample_stops_without_motion(monkeypatch):
    class StalePoller:
        def __init__(self, *_args, **_kwargs):
            pass

        def start(self):
            pass

        def stop(self):
            pass

        def latest_after(self, _sequence, _timeout):
            return PollSnapshot(0, None, None, None, 0, None)

    monkeypatch.setattr("waferbot.sensing.follower.LatestIRPoller", StalePoller)
    transport = MockI2CTransport(line_sensor_byte=0x03)
    robot = Robot(transport, RobotConfig().with_speed_limit(5), watchdog=False)
    robot.arm()
    with pytest.raises(SensorError, match="no fresh IR sample"):
        PIDFollower(robot, replace(PIDConfig(), enabled=True)).run(
            EdgeState.BLACK_LEFT, duration_s=0.1
        )
    assert not any(payload[2] > 0 for payload in transport.motor_payloads)
    robot.close()


def test_stop_latch_is_written_even_if_robot_config_is_missing(tmp_path):
    runtime = tmp_path / "runtime"
    code = cli_main([
        "stop", "--robot-config", str(tmp_path / "missing.json"),
        "--runtime-dir", str(runtime),
    ])
    assert code == 3
    assert ProcessGuard(runtime).stop_requested()
