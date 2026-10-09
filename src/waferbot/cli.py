"""Small physical CLI for the PID-only robot project."""

from __future__ import annotations

import argparse
import json
import sys

from . import __version__
from .config import RobotConfig
from .errors import WaferbotError
from .pidconfig import PIDConfig
from .process import ProcessGuard
from .robot import Robot
from .sensing.edge import EdgeState
from .sensing.follower import PIDFollower


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(description="Waferbot PID edge follower")
    root.add_argument("--version", action="version", version=f"waferbot {__version__}")
    commands = root.add_subparsers(dest="command", required=True)
    sensors = commands.add_parser("sensors", help="read IR sensors without moving")
    sensors.add_argument("--physical", action="store_true", required=True)
    sensors.add_argument("--robot-config", required=True)
    follow = commands.add_parser("follow", help="run a bounded physical PID test")
    follow.add_argument("--physical", action="store_true", required=True)
    follow.add_argument("--robot-config", required=True)
    follow.add_argument("--pid-config", required=True)
    follow.add_argument("--edge", choices=("black-left", "black-right"), required=True)
    follow.add_argument("--duration", type=float, default=3.0)
    follow.add_argument("--control-log-csv")
    follow.add_argument("--runtime-dir")
    stop = commands.add_parser("stop", help="latch stop and command zero wheels")
    stop.add_argument("--robot-config", required=True)
    stop.add_argument("--runtime-dir")
    clear = commands.add_parser("clear-stop", help="clear stop latch while idle")
    clear.add_argument("--runtime-dir")
    return root


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        if args.command == "sensors":
            config = RobotConfig.load_json(args.robot_config)
            with Robot.connected(config, watchdog=False) as robot:
                print(json.dumps(robot.read_line_sensors().as_dict(), indent=2))
            return 0
        guard = ProcessGuard(args.runtime_dir)
        if args.command == "clear-stop":
            if not guard.owner_lock_available():
                raise ValueError("a motion session is still running")
            print("stop latch cleared" if guard.clear_stop_request() else "stop latch was clear")
            return 0
        if args.command == "stop":
            guard.request_stop("operator stop")
            config = RobotConfig.load_json(args.robot_config)
            with Robot.connected(config, watchdog=False, bus_guard=guard.bus_guard) as robot:
                robot.stop()
            print("stop latched; all wheels commanded zero")
            return 0
        config = RobotConfig.load_json(args.robot_config)
        pid_config = PIDConfig.load_json(args.pid_config)
        if not pid_config.enabled:
            raise ValueError("PID config is disabled; enable a verified operator copy")
        if guard.stop_requested():
            raise ValueError("stop latch is set; use clear-stop while idle before following")
        if input("Physical wheels may move. Type yes to run: ").strip() != "yes":
            print("cancelled")
            return 2
        guard.acquire_ownership("pid_follow")
        try:
            with Robot.connected(
                config, stop_check=guard.stop_requested, bus_guard=guard.bus_guard,
            ) as robot:
                robot.arm()
                edge = EdgeState.BLACK_LEFT if args.edge == "black-left" else EdgeState.BLACK_RIGHT
                result = PIDFollower(
                    robot, pid_config, stop_requested=guard.stop_requested,
                ).run(edge, duration_s=args.duration, log_csv=args.control_log_csv)
            print(json.dumps(vars(result), indent=2))
            return 0 if result.stop_reason == "MAX_DURATION" else 3
        finally:
            guard.release_ownership()
    except (ValueError, OSError, WaferbotError) as exc:
        print(f"waferbot: {exc}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
