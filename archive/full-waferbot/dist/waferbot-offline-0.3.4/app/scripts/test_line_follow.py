#!/usr/bin/env python3
"""Follow either tape boundary and report what happened.

Two modes:

* default (no hardware): the real ``EdgeFollower`` drives the deterministic
  kinematic tape model, so the trajectory, the boundary estimate, and the
  convergence metrics are available on a laptop. These are *synthetic* results.
* ``--physical``: the same controller drives the real chassis, with an explicit
  consent prompt, a bounded duration, and a mandatory stop. Nothing is claimed
  about hardware that was not run.

Examples::

    python scripts/test_line_follow.py --edge both
    python scripts/test_line_follow.py --edge black-left --pitch-mm 20
    python scripts/test_line_follow.py --physical \
        --config ~/.config/waferbot/robot.json \
        --nav-config ~/.config/waferbot/nav.json \
        --edge black-left --duration 3
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from waferbot.config import RobotConfig  # noqa: E402
from waferbot.navconfig import NavConfig  # noqa: E402
from waferbot.sensing.edge import EdgeState  # noqa: E402
from waferbot.simulation import Pose, TapeModelConfig, run_closed_loop  # noqa: E402

EDGES = {"black-left": EdgeState.BLACK_LEFT, "black-right": EdgeState.BLACK_RIGHT}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--edge",
        choices=[*EDGES, "both"],
        default="both",
        help="which tape boundary to follow (default: both)",
    )
    parser.add_argument("--physical", action="store_true", help="run on real hardware")
    parser.add_argument("--config", help="robot configuration JSON (physical runs)")
    parser.add_argument("--nav-config", help="navigation configuration JSON")
    parser.add_argument("--runtime-dir", help="shared CLI ownership/stop directory")
    parser.add_argument("--log-csv", help="telemetry CSV path (physical runs)")
    parser.add_argument(
        "--duration", type=float, default=3.0, help="physical seconds per edge"
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=200,
        help="model only: control iterations per edge (20 iterations = 1 s)",
    )
    parser.add_argument("--speed", type=int, default=None, help="override base speed")
    parser.add_argument(
        "--pitch-mm",
        type=float,
        default=20.0,
        help="model only: sensor pitch in millimetres (synthetic assumption)",
    )
    parser.add_argument(
        "--tape-mm",
        type=float,
        default=30.0,
        help="model only: tape width in millimetres (synthetic assumption)",
    )
    parser.add_argument(
        "--offset-mm",
        type=float,
        default=5.0,
        help="model only: initial lateral offset in millimetres",
    )
    parser.add_argument(
        "--heading-deg",
        type=float,
        default=3.0,
        help="model only: initial heading offset in degrees",
    )
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    return parser


def model_config(args: argparse.Namespace) -> TapeModelConfig:
    return TapeModelConfig(
        sensor_pitch_m=args.pitch_mm / 1000.0,
        tape_width_m=args.tape_mm / 1000.0,
        sensor_detection_width_m=0.006,
    )


def run_model(args: argparse.Namespace, edge: EdgeState) -> dict:
    follow = None
    if args.nav_config:
        follow = NavConfig.load_json(args.nav_config).follow
    if args.speed is not None:
        if follow is None:
            from waferbot.navconfig import FollowConfig

            follow = FollowConfig()
        follow = type(follow)(**{**follow.__dict__, "base_speed": args.speed})
    # One control period (50 ms) per iteration of the synthetic model.
    iterations = max(20, int(args.iterations))
    sign = -1.0 if edge is EdgeState.BLACK_LEFT else 1.0
    pose = Pose(
        x=0.0,
        y=sign * (args.tape_mm / 2000.0) + args.offset_mm / 1000.0,
        heading=math.radians(args.heading_deg),
    )
    run = run_closed_loop(
        edge=edge,
        model_config=model_config(args),
        initial_pose=pose,
        iterations=iterations,
        follow_config=follow,
        robot_config=RobotConfig.load_json(args.config) if args.config else None,
    )
    payload = dict(run.metrics.as_dict())
    payload.update(
        {
            "mode": "synthetic tape model",
            "edge": edge.value,
            "iterations": iterations,
            "simulated_seconds": iterations * model_config(args).step_s,
            "note": (
                "synthetic kinematic model evidence, not a hardware measurement; "
                f"sensor pitch {args.pitch_mm:g} mm, tape {args.tape_mm:g} mm"
            ),
        }
    )
    return payload


def run_physical(args: argparse.Namespace, edge: EdgeState) -> int:
    """Use exactly the CLI's physical session, consent and stop controls."""
    from waferbot.cli import main as cli_main

    root = Path(__file__).resolve().parent.parent
    argv = [
        "follow", "--physical", "--edge", edge.value.lower().replace("_", "-"),
        "--duration", str(args.duration),
        "--config", args.config or str(root / "config/robot.first-run.json"),
        "--nav-config", args.nav_config or str(root / "config/nav.first-run.json"),
    ]
    for name in ("speed", "runtime_dir", "log_csv"):
        value = getattr(args, name)
        if value is not None:
            argv.extend(["--" + name.replace("_", "-"), str(value)])
    if args.json:
        argv.append("--json")
    return cli_main(argv)


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.physical:
        if args.edge == "both":
            parser.error("physical runs require --edge black-left or black-right; reposition between runs")
        if not math.isfinite(args.duration) or args.duration <= 0:
            parser.error("--duration must be finite and positive")
        return run_physical(args, EDGES[args.edge])
    edges = list(EDGES.values()) if args.edge == "both" else [EDGES[args.edge]]
    reports = []
    exit_code = 0
    for edge in edges:
        report = run_model(args, edge)
        reports.append(report)
        if not args.json:
            def mm(value):
                return "n/a" if value is None else f"{value * 1000:.1f} mm"

            tail = report["tail_estimate_rms_pitches"]
            tail_text = "n/a" if tail is None else f"{tail:.2f} pitch"
            print(
                f"{edge.value}: stop={report['stop_reason']} "
                f"converged={report['converged']} "
                f"rms={mm(report['rms_lateral_m'])} "
                f"max={mm(report['max_lateral_m'])} "
                f"tail_error={tail_text}"
            )
        if report.get("stop_reason") in {"LINE_LOST", "ACQUISITION_FAILED", "FAULT"}:
            exit_code = 1
    if args.json:
        print(json.dumps(reports, indent=2))
    else:
        print(
            "\nSynthetic model evidence only: these numbers come from the "
            "kinematic tape model, not from the Raspberry Pi."
        )
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
