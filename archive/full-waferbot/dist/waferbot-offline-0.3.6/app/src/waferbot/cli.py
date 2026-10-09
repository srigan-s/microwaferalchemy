"""`waferbot` command line interface.

Design rules:

* Mock is the default. Real hardware requires ``--physical`` *and* typing
  ``yes`` at the prompt; there is no flag that skips the prompt.
* Prompts never happen while the wheels are turning.
* Motion sessions take process ownership and honour an external stop latch, so
  `waferbot stop` genuinely stops a running controller.
* Every command that plans or executes refuses to invent measurements: an
  uncalibrated map, an unmeasured geometry, or an uncalibrated speed leaves the
  operation blocked with an explanation.

Exit codes: 0 success, 2 usage/config problem, 3 refused or faulted.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from collections.abc import Callable
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

from . import __version__
from .calibration import (
    CrossingSweep,
    WheelCalibrator,
    calibrate_channels,
    compute_counts_to_mps,
    format_samples,
    read_samples,
    speed_calibration_instructions,
)
from .config import RobotConfig
from .errors import (
    AuthorizationError,
    ConfigError,
    ExecutionError,
    I2CError,
    LocalizationError,
    MapError,
    PlanningError,
    SafetyError,
    SensorError,
    WaferbotError,
)
from .edge_angle_experiment import (
    append_summary,
    minimum_confirmed_angle,
    run_straight_crossing,
    straight_wheels,
    summarize_angles,
)
from .ir_experiment import run_ir_strafe
from .localization import (
    Localization,
    ManualArrivalMonitor,
    MockArrivalMonitor,
)
from .navconfig import NavConfig
from .nav.executor import RouteExecutor
from .nav.graph import TrackMap
from .nav.planner import Planner
from .process import ProcessGuard
from .robot import Robot
from .safety import FaultCode, SafetyState
from .sensing.edge import EdgeState, edge_byte_for
from .sensing.follower import EdgeFollower, FollowStopReason
from .sensing.switching import EdgeSwitcher, SwitchAuthorization
from .telemetry import open_telemetry

DEFAULT_MAP = "maps/example_track.json"
EXIT_OK = 0
EXIT_USAGE = 2
EXIT_REFUSED = 3

#: Injection seams used by tests and embedding code. They must stay `None` in
#: normal operation: a physical transport is only ever opened through
#: :func:`_open_transport`, and prompts only through :func:`_prompt`.
PHYSICAL_TRANSPORT_FACTORY: Callable[[RobotConfig], Any] | None = None
INTERACTIVE_OVERRIDE: bool | None = None


# ---------------------------------------------------------------------------
# session plumbing
# ---------------------------------------------------------------------------


@dataclass
class Session:
    args: argparse.Namespace
    config: RobotConfig
    nav: NavConfig
    robot: Robot
    track: TrackMap | None
    guard: ProcessGuard
    telemetry: Any
    transport: Any
    mock: bool
    sensor_script: Any = None
    read_only: bool = False
    obstacle_monitor: Any = None
    owns_bus: bool = False

    def close(self) -> None:
        try:
            if self.obstacle_monitor is not None:
                self.obstacle_monitor.stop()
            if self.read_only:
                # A read-only diagnostic session must not command the motors,
                # not even as cleanup.
                closer = getattr(self.transport, "close", None)
                if closer is not None:
                    closer()
            else:
                self.robot.close()
        finally:
            self.telemetry.close()
            if self.owns_bus:
                self.guard.release_ownership()

    @property
    def hardware(self) -> bool:
        return not self.mock


def resolve_map_path(path: str | os.PathLike[str] | None) -> Path:
    """Resolve a map path, falling back to the resource shipped in the wheel."""
    candidate = Path(path) if path else Path(DEFAULT_MAP)
    if candidate.is_file():
        return candidate
    if Path(path or DEFAULT_MAP).name == Path(DEFAULT_MAP).name:
        packaged = Path(__file__).resolve().parent / "data" / "example_track.json"
        if packaged.is_file():
            return packaged
    raise MapError(f"track map not found: {candidate}")


def _open_transport(config: RobotConfig):
    if PHYSICAL_TRANSPORT_FACTORY is not None:
        return PHYSICAL_TRANSPORT_FACTORY(config)
    from .hardware.transport import Smbus2Transport

    return Smbus2Transport(config.motor.bus)


def _is_interactive() -> bool:
    if INTERACTIVE_OVERRIDE is not None:
        return bool(INTERACTIVE_OVERRIDE)
    return bool(sys.stdin.isatty())


def _load_nav(args: argparse.Namespace) -> NavConfig:
    if getattr(args, "nav_config", None):
        nav = NavConfig.load_json(args.nav_config)
    else:
        nav = NavConfig()
    overrides = {}
    if getattr(args, "speed", None) is not None:
        overrides["base_speed"] = int(args.speed)
    if getattr(args, "rate_hz", None) is not None:
        overrides["rate_hz"] = float(args.rate_hz)
    if overrides:
        nav = nav.with_follow(**overrides)
    return nav.validate()


def _load_config(args: argparse.Namespace) -> RobotConfig:
    if getattr(args, "config", None):
        return RobotConfig.load_json(args.config)
    return RobotConfig().validate()


def _map_path(args: argparse.Namespace, nav: NavConfig) -> str:
    return getattr(args, "map", None) or nav.map_path or DEFAULT_MAP


def _telemetry_path(args: argparse.Namespace, nav: NavConfig) -> str | None:
    return getattr(args, "log_csv", None) or nav.telemetry_csv


def _prompt(text: str) -> str:
    try:
        return input(text)
    except EOFError:
        return ""


def _sleep_for(session: Session):
    """Mock sessions do not need to wait for wall-clock time; hardware does."""
    if session.mock:
        return lambda _seconds: None
    return time.sleep


def _require_interactive(args: argparse.Namespace, action: str) -> None:
    """Physical motion needs an interactive, explicit confirmation."""
    if not _is_interactive():
        raise WaferbotError(
            f"{action} on real hardware needs an interactive confirmation; "
            "run it from a terminal (mock mode needs no prompt)"
        )


def _confirm_physical(args: argparse.Namespace, action: str) -> None:
    _require_interactive(args, action)
    answer = _prompt(
        f"About to {action} on REAL hardware (Yahboom Raspbot V2).\n"
        "Wheels must be clear of people and obstacles. Type 'yes' to continue: "
    )
    if answer.strip().lower() != "yes":
        raise WaferbotError(f"physical {action} was not confirmed")


def _build_session(
    args: argparse.Namespace,
    command: str,
    *,
    motion: bool,
    floor_motion: bool = False,
    require_map: bool = False,
    read_only: bool = False,
    initial_edge: EdgeState = EdgeState.BLACK_LEFT,
) -> Session:
    config = _load_config(args)
    nav = _load_nav(args)
    guard = ProcessGuard(getattr(args, "runtime_dir", None))
    telemetry = open_telemetry(
        _telemetry_path(args, nav), counts_to_mps=nav.counts_to_mps
    )
    mock = not getattr(args, "physical", False)

    if mock:
        from .hardware.mock import MockI2CTransport, MockLineSensorScript

        script = MockLineSensorScript(edge_byte_for(initial_edge, config.sensor))
        transport = MockI2CTransport(line_sensor_provider=script)
        robot = Robot(
            transport,
            config,
            telemetry=telemetry,
            watchdog=False,
            stop_check=guard.stop_requested,
        )
        transport_mock = True
    else:
        if motion:
            _require_interactive(args, command)
            pending = config.unverified_components()
            if floor_motion and pending and not getattr(
                args, "ack_verified_config", False
            ):
                raise SafetyError(
                    f"{command}: floor operation refused because these hardware "
                    "assumptions are not verified in configuration:\n  - "
                    + "\n  - ".join(pending)
                    + "\nRun `waferbot calibrate wheels` / `waferbot calibrate "
                    "sensors` and set `verified: true`, or pass "
                    "--ack-verified-config to acknowledge them yourself."
                )
            if floor_motion and pending:
                print(
                    "warning: proceeding with unverified hardware assumptions "
                    "acknowledged by --ack-verified-config:\n  - "
                    + "\n  - ".join(pending),
                    file=sys.stderr,
                )
            _confirm_physical(args, command)
        if motion:
            if guard.stop_requested():
                raise SafetyError(
                    "a stop was requested by another process; run "
                    "`waferbot stop --clear` once you have checked the robot"
                )
            guard.acquire_ownership(command)
        try:
            transport = _open_transport(config)
        except Exception:
            telemetry.close()
            guard.release_ownership()
            raise
        robot = Robot(
            transport,
            config,
            stop_check=guard.stop_requested if motion else None,
            bus_guard=(lambda: guard.bus_guard(1.0)) if motion else None,
            telemetry=telemetry,
        )
        script = None
        transport_mock = False

    track = None
    if require_map:
        try:
            track = TrackMap.load(resolve_map_path(_map_path(args, nav)))
        except Exception:
            telemetry.close()
            robot.close()
            guard.release_ownership()
            raise

    session = Session(
        args=args,
        config=config,
        nav=nav,
        robot=robot,
        track=track,
        guard=guard,
        telemetry=telemetry,
        transport=transport,
        mock=mock,
        sensor_script=script,
        read_only=read_only,
        owns_bus=motion and not mock,
    )
    if motion:
        if guard.stop_requested():
            session.close()
            raise SafetyError(
                "a stop was requested by another process; run "
                "`waferbot stop --clear` once you have checked the robot"
            )
        _attach_obstacle_monitor(session, args)
    return session


def _attach_obstacle_monitor(session: Session, args: argparse.Namespace) -> None:
    """Optional, opt-in obstacle monitoring for physical sessions."""
    threshold = getattr(args, "obstacle_distance_mm", None)
    if threshold is None:
        return
    if session.mock:
        print(
            "--obstacle-distance-mm is ignored in mock mode (no ranging hardware)",
            file=sys.stderr,
        )
        return
    from .hardware.ultrasonic import ObstacleMonitor, UltrasonicSensor

    sensor = UltrasonicSensor(session.transport, address=session.config.motor.address)

    def on_obstacle(distance_mm: int) -> None:
        try:
            session.robot.emergency_stop(
                FaultCode.OBSTACLE_DETECTED, f"obstacle at {distance_mm} mm"
            )
        except I2CError as exc:  # pragma: no cover - hardware path
            print(f"obstacle stop failed: {exc}", file=sys.stderr)

    monitor = ObstacleMonitor(sensor, on_obstacle, threshold_mm=int(threshold))
    session.obstacle_monitor = monitor
    try:
        monitor.start()
        # Check once before any command can arm the motors. The monitor thread
        # otherwise waits one interval before its first reading.
        monitor.poll_once()
    except BaseException:
        session.close()
        raise


def _printer(args: argparse.Namespace):
    as_json = bool(getattr(args, "json", False))

    def emit(payload: Any, text: str) -> None:
        if as_json:
            print(json.dumps(payload, indent=2, default=str))
        else:
            print(text)

    return emit


# ---------------------------------------------------------------------------
# commands
# ---------------------------------------------------------------------------


def cmd_sensors(args: argparse.Namespace) -> int:
    # Read-only: no map, and cleanup must not command the motors.
    session = _build_session(args, "sensors", motion=False, read_only=True)
    emit = _printer(args)
    try:
        samples = read_samples(session.robot, args.samples, interval_s=args.interval)
        emit(
            {"samples": [sample.as_dict() for sample in samples]},
            format_samples(samples),
        )
        return EXIT_OK
    finally:
        session.close()


def cmd_ir_strafe(args: argparse.Namespace) -> int:
    """Log the line sensor while making a bounded rightward floor strafe."""
    config = _load_config(args)
    if not 1 <= args.speed_pwm <= config.motor.max_speed:
        raise ValueError(f"--speed-pwm must be 1..{config.motor.max_speed}")
    if not math.isfinite(args.sample_rate_hz) or not 1 <= args.sample_rate_hz <= 500:
        raise ValueError("--sample-rate-hz must be between 1 and 500")
    if not math.isfinite(args.duration) or not 0 < args.duration <= 10:
        raise ValueError("--duration must be between 0 and 10 seconds")
    session = _build_session(args, "ir-strafe", motion=True, floor_motion=True)
    try:
        session.robot.arm()
        result = run_ir_strafe(
            session.robot,
            args.output_csv,
            speed=args.speed_pwm,
            rate_hz=args.sample_rate_hz,
            duration_s=args.duration,
            stop_requested=session.guard.stop_requested,
            sleep=_sleep_for(session),
        )
    finally:
        session.close()
    _printer(args)(result, json.dumps(result, indent=2))
    return EXIT_OK if result["stop_reason"] == "DURATION" else EXIT_REFUSED


def _crossing_angles(value: str) -> list[float]:
    """Accept comma-separated angles or inclusive start:stop:step notation."""
    try:
        if ":" in value:
            start, stop, step = (float(part) for part in value.split(":"))
            if not all(math.isfinite(item) for item in (start, stop, step)) or step <= 0:
                raise ValueError
            count = int((stop - start) / step) + 1
            if not 1 <= count <= 30:
                raise ValueError
            angles = [start + index * step for index in range(count)]
        else:
            angles = [float(part) for part in value.split(",")]
        if not angles or len(angles) > 30 or len(set(angles)) != len(angles):
            raise ValueError
        if any(not math.isfinite(angle) or not 0 < angle < 90 for angle in angles):
            raise ValueError
        return angles
    except ValueError as exc:
        raise ValueError("--angles must be unique values in (0,90), e.g. 10:80:10") from exc


def cmd_edge_angle_sweep(args: argparse.Namespace) -> int:
    """Run repeated constant-wheel crossings with per-read IR traces."""
    config = _load_config(args)
    angles = _crossing_angles(args.angles)
    if not 1 <= args.speed_pwm <= config.motor.max_speed:
        raise ValueError(f"--speed-pwm must be 1..{config.motor.max_speed}")
    if not 1 <= args.attempts <= 10:
        raise ValueError("--attempts must be 1..10")
    if not math.isfinite(args.duration) or not 0 < args.duration <= 10:
        raise ValueError("--duration must be between 0 and 10 seconds")
    if not math.isfinite(args.confirm_ms) or not 0 < args.confirm_ms <= 1000:
        raise ValueError("--confirm-ms must be between 0 and 1000")
    rate = None if args.sample_rate_hz == "max" else float(args.sample_rate_hz)
    if rate is not None and (not math.isfinite(rate) or not 1 <= rate <= 500):
        raise ValueError("--sample-rate-hz must be max or a number from 1 to 500")
    output_dir = Path(args.output_dir)
    summary_path = output_dir / "summary.csv"
    if summary_path.exists():
        raise ValueError(f"{summary_path} already exists; choose a new --output-dir")
    vectors = {angle: straight_wheels(angle, args.speed_pwm) for angle in angles}
    session = _build_session(args, "edge-angle-sweep", motion=True, floor_motion=True)
    results: list[dict[str, object]] = []
    try:
        session.robot.arm()
        for angle in angles:
            wheels, realized = vectors[angle]
            for attempt in range(1, args.attempts + 1):
                session.robot.stop()
                if session.guard.stop_requested():
                    break
                if not session.mock:
                    answer = _prompt(
                        f"WHEELS STOPPED: place the BLACK_LEFT boundary between S2/S3 "
                        f"for {angle:g} deg (commanded {realized:.1f} deg, wheels {wheels}), "
                        f"attempt {attempt}/{args.attempts}. Clear the forward-left path; "
                        "type 'ready' to run: "
                    )
                    if answer.strip().lower() != "ready":
                        raise WaferbotError("edge-angle sweep repositioning was not confirmed")
                trace_path = output_dir / f"angle-{angle:g}-attempt-{attempt}.csv"
                result = run_straight_crossing(
                    session.robot, trace_path,
                    angle_deg=angle,
                    speed=args.speed_pwm,
                    duration_s=args.duration,
                    sample_rate_hz=rate,
                    confirm_ms=args.confirm_ms,
                    stop_requested=session.guard.stop_requested,
                    sleep=_sleep_for(session),
                )
                results.append(result)
                append_summary(summary_path, result, attempt)
                if result["stop_reason"] in {"STOP_REQUESTED", "FAULT", "SOURCE_NOT_ACQUIRED"}:
                    break
            if len(results) < angles.index(angle) * args.attempts + args.attempts:
                break
    finally:
        session.close()
    minimum_requested = minimum_confirmed_angle(results, args.attempts)
    angle_results = summarize_angles(results, angles, args.attempts)
    reliable = [
        row for row in angle_results
        if row["all_repeats_confirmed"] and row["median_switch_time_ms"] is not None
    ]
    fastest = min(
        reliable,
        key=lambda row: (row["median_switch_time_ms"], row["requested_angle_deg"]),
        default=None,
    )
    report = {
        "summary_csv": str(summary_path),
        "attempts_completed": len(results),
        "attempts_requested": len(angles) * args.attempts,
        "minimum_confirmed_requested_angle_deg": minimum_requested,
        "minimum_confirmed_commanded_angle_deg": (
            None if minimum_requested is None else vectors[minimum_requested][1]
        ),
        "fastest_reliable_requested_angle_deg": (
            None if fastest is None else fastest["requested_angle_deg"]
        ),
        "fastest_reliable_commanded_angle_deg": (
            None if fastest is None else fastest["commanded_angle_deg"]
        ),
        "angle_results": angle_results,
        "note": (
            "Minimum is the smallest tested angle with every sensor trial confirmed. "
            "PWM quantization can duplicate angles; destination/location and "
            "physical trajectory are not verified."
        ),
    }
    _printer(args)(report, json.dumps(report, indent=2))
    return EXIT_OK if len(results) == len(angles) * args.attempts else EXIT_REFUSED


def cmd_motor_test(args: argparse.Namespace) -> int:
    session = _build_session(args, "motor-test", motion=True)
    emit = _printer(args)
    interactive = bool(sys.stdin.isatty())
    try:
        if not session.mock:
            answer = _prompt(
                "WHEELS OFF THE GROUND: support the chassis so no wheel touches "
                "the floor, then type 'lifted' to continue: "
            )
            if answer.strip().lower() != "lifted":
                raise WaferbotError("motor test requires the wheels to be lifted")

        def ask(prompt: str) -> str:
            if not interactive:
                print(f"[mock] {prompt} -> unknown", file=sys.stderr)
                return "unknown"
            return _prompt(prompt + " ")

        session.robot.arm()
        calibrator = WheelCalibrator(
            session.robot,
            ask=ask,
            telemetry=session.telemetry,
            sleep=_sleep_for(session),
        )
        wheels = None if args.wheel is None else [args.wheel]
        report = calibrator.run(
            speed=args.speed,
            duration_s=args.duration,
            wheels=wheels,
            directions=args.directions,
        )
        payload = report.as_dict()
        payload["wheel_signatures"] = "see waferbot.kinematics.WHEEL_SIGNATURES"
        payload["note"] = (
            "Wheel order/sign mapping stays UNCONFIRMED until every probe "
            "answers yes on real hardware."
        )
        emit(payload, json.dumps(payload, indent=2))
        return EXIT_OK
    finally:
        session.close()


def cmd_calibrate(args: argparse.Namespace) -> int:
    target = args.calibration
    if target == "speed":
        return _calibrate_speed(args)
    if target == "sensors":
        return _calibrate_sensors(args)
    if target == "wheels":
        return cmd_motor_test(args)
    if target == "crossing":
        return _calibrate_crossing(args)
    raise WaferbotError(f"unknown calibration {target!r}")


def _calibrate_speed(args: argparse.Namespace) -> int:
    from .calibration import SpeedCalibrationReport, SpeedSample

    if args.instructions:
        print(speed_calibration_instructions())
        return EXIT_OK
    if args.counts is None or args.distance_m is None or args.duration_s is None:
        print(speed_calibration_instructions())
        return EXIT_USAGE
    report = SpeedCalibrationReport(
        samples=[
            SpeedSample(
                counts=float(args.counts),
                distance_m=float(args.distance_m),
                duration_s=float(args.duration_s),
            )
        ]
    )
    payload = report.as_dict()
    print(json.dumps(payload, indent=2))
    print(speed_calibration_instructions())
    return EXIT_OK


def _calibrate_sensors(args: argparse.Namespace) -> int:
    session = _build_session(args, "calibrate sensors", motion=False)
    try:
        interactive = sys.stdin.isatty() and not session.mock

        def ask(prompt: str) -> str:
            if not interactive:
                print(f"[mock] {prompt} -> skip", file=sys.stderr)
                return "skip"
            return _prompt(prompt + " ")

        report = calibrate_channels(
            session.robot, ask=ask, samples_per_channel=args.samples
        )
        print(json.dumps(report.as_dict(), indent=2))
        return EXIT_OK
    finally:
        session.close()


def _calibrate_crossing(args: argparse.Namespace) -> int:
    session = _build_session(
        args, "calibrate crossing", motion=True, floor_motion=True
    )
    try:
        if not session.nav.geometry.measured:
            print(
                "Crossing sweep needs measured geometry (tape width, sensor "
                "spacing, detection width, clearance, switching speed, sampling "
                "rate). Fill nav.geometry and set measured=true first."
            )
            return EXIT_REFUSED
        angles = [float(value) for value in args.angles.split(",") if value.strip()]
        if not angles:
            raise WaferbotError("--angles needs at least one value")
        source = EdgeState.from_name(args.from_edge)
        target = EdgeState.from_name(args.to_edge)
        session.robot.arm()
        if session.sensor_script is not None:
            session.sensor_script.set_byte(edge_byte_for(source, session.config.sensor))
        mock = session.mock

        attempt_counter = {"n": 0}

        def attempt(angle: float):
            attempt_counter["n"] += 1
            if mock and session.sensor_script is not None:
                session.sensor_script.set_byte(
                    edge_byte_for(source, session.config.sensor)
                )
                session.sensor_script.schedule_byte(
                    # Establishment plus the post-confirmation recheck read the
                    # source pattern before the crossed edge appears.
                    edge_byte_for(target, session.config.sensor),
                    after_reads=8,
                )
            if not mock:
                # Every real attempt starts from a stopped, explicitly
                # repositioned robot: --location is a requested identifier, not
                # observed localization.
                session.robot.stop()
                if not _manual_confirm(
                    f"Attempt {attempt_counter['n']} at {angle} deg: reposition "
                    f"the robot at the source edge/location {args.location} "
                    "(motors are stopped), then confirm"
                ):
                    raise WaferbotError(
                        "crossing sweep attempt was not confirmed; stopping"
                    )
            config = session.nav.switch
            switcher = EdgeSwitcher(
                session.robot,
                config=replace(config, mode="diagonal", crossing_angle_deg=angle),
                geometry=session.nav.geometry,
                telemetry=session.telemetry,
                location_provider=_fixed_localizer(args.location) if mock else None,
                confirm=(lambda _text: True) if mock else _manual_confirm,
                sleep=_sleep_for(session),
                counts_to_mps=session.nav.counts_to_mps,
                require_distance_bound=not mock,
            )
            authorization = SwitchAuthorization(
                authorized=True,
                route_id="calibration",
                source_node=args.location,
                source_edge=source,
                target_edge=target,
                location_id=args.location,
                destination_location_id=args.location,
                issued_at=time.monotonic(),
            )
            return switcher.switch_edge(
                source,
                target,
                authorization=authorization,
                location_id=args.location,
                destination_monitor=(
                    _FixedMonitor(args.location) if mock else None
                ),
            )

        sweep = CrossingSweep(attempt, csv_path=args.csv)
        report = sweep.run(angles, attempts_per_angle=args.attempts)
        print(json.dumps(report.as_dict(), indent=2))
        return EXIT_OK if report.best_angle_deg is not None else EXIT_REFUSED
    finally:
        session.close()


def _fixed_localizer(node: str):
    def provider(expected: str):
        timestamp = time.monotonic()
        return Localization(
            node_id=expected,
            marker_id=expected,
            confidence=1.0,
            timestamp=timestamp,
            source="fixed",
        )

    return provider


class _FixedMonitor:
    """Arrival monitor that reports a single, already-confirmed location."""

    def __init__(self, node: str) -> None:
        self.node = node

    def requires_stop(self) -> bool:
        return False

    def check(self, expected_node: str):
        if expected_node != self.node:
            return None
        return Localization(
            node_id=expected_node,
            marker_id=expected_node,
            confidence=1.0,
            timestamp=time.monotonic(),
            source="fixed",
        )


def cmd_follow(args: argparse.Namespace) -> int:
    edge = EdgeState.from_name(args.edge)
    if getattr(args, "physical", False):
        _require_interactive(args, "follow")
    if getattr(args, "physical", False) and not _load_nav(args).follow.controller_enabled:
        raise SafetyError(
            "physical PID following is disabled; set follow.controller_enabled=true "
            "in a reviewed nav configuration after bench validation"
        )
    session = _build_session(
        args,
        f"follow {edge.value}",
        motion=True,
        floor_motion=True,
        initial_edge=edge,
    )
    emit = _printer(args)
    try:
        session.robot.arm()
        follower = EdgeFollower(
            session.robot,
            config=session.nav.follow,
            telemetry=session.telemetry,
            sleep=_sleep_for(session),
            use_async_poller=session.hardware,
            control_log_csv=args.control_log_csv,
        )
        result = follower.follow_edge(
            edge,
            max_duration_s=args.duration,
            max_iterations=args.iterations,
            max_speed_pwm=5 if session.hardware else None,
        )
    finally:
        session.close()
    # Report success only after the session's final stop/close succeeds.
    emit(result.as_dict(), json.dumps(result.as_dict(), indent=2))
    return EXIT_OK if result.completed else EXIT_REFUSED


def cmd_switch(args: argparse.Namespace) -> int:
    source = EdgeState.from_name(args.from_edge)
    target = EdgeState.from_name(args.to_edge)
    if not args.authorize:
        print(
            "refused: edge switching requires route authorization. Pass "
            "--authorize with --location and --route-id from the route executor."
        )
        return EXIT_REFUSED
    session = _build_session(
        args,
        f"switch {source.value}->{target.value}",
        motion=True,
        floor_motion=True,
        initial_edge=source,
    )
    try:
        session.robot.arm()
        if session.sensor_script is not None:
            session.sensor_script.set_byte(
                edge_byte_for(source, session.config.sensor)
            )
            session.sensor_script.schedule_byte(
                # Enough reads for establishment plus the post-confirmation
                # recheck before the crossed edge appears.
                edge_byte_for(target, session.config.sensor),
                after_reads=8,
            )
        authorization = SwitchAuthorization(
            authorized=True,
            route_id=args.route_id,
            source_node=args.location,
            source_edge=source,
            target_edge=target,
            location_id=args.location,
            destination_location_id=args.destination or args.location,
            issued_at=time.monotonic(),
        )
        # Mock keeps deterministic stand-ins; hardware must use real evidence:
        # an operator prompt (with the wheels stopped) or a marker reader.
        if session.mock:
            location_provider = _fixed_localizer(args.location)
            monitor = _FixedMonitor(args.destination or args.location)
            confirm = lambda _text: True  # noqa: E731 - mock stand-in only
        else:
            location_provider = None
            monitor = None
            confirm = _manual_confirm
        switcher = EdgeSwitcher(
            session.robot,
            config=session.nav.switch,
            geometry=session.nav.geometry,
            telemetry=session.telemetry,
            location_provider=location_provider,
            confirm=confirm,
            sleep=_sleep_for(session),
            counts_to_mps=session.nav.counts_to_mps,
            require_distance_bound=not session.mock,
        )
        result = switcher.switch_edge(
            source,
            target,
            authorization=authorization,
            location_id=args.location,
            destination_monitor=monitor,
        )
        payload = result.as_dict()
        emit = _printer(args)
        emit(payload, json.dumps(payload, indent=2))
        return EXIT_OK if result.completed else EXIT_REFUSED
    finally:
        session.close()


def cmd_map(args: argparse.Namespace) -> int:
    nav = _load_nav(args)
    path = _map_path(args, nav)
    resolved = resolve_map_path(path)
    track = TrackMap.load(resolved)
    payload = {
        "path": str(resolved),
        "name": track.name,
        "is_example": track.is_example,
        "physical_validated": track.physical_validated,
        "edge_side_map": dict(track.edge_side_map),
        "nodes": len(track.nodes),
        "edges": len(track.edges),
        "enabled_edges": len(track.enabled_edges),
        "max_speed_mps": track.max_speed_mps,
        "adjacency": track.adjacency(),
        "node_types": sorted({node.node_type.value for node in track.nodes}),
        "actions": sorted({edge.action.value for edge in track.edges}),
        "physical_ready": _physical_ready(track),
    }
    _printer(args)(payload, json.dumps(payload, indent=2))
    return EXIT_OK if (payload["physical_ready"] or not args.require_physical) else EXIT_REFUSED


def _physical_ready(track: TrackMap) -> bool:
    try:
        track.require_physical_ready()
    except MapError:
        return False
    return True


def _plan(args: argparse.Namespace):
    nav = _load_nav(args)
    track = TrackMap.load(resolve_map_path(_map_path(args, nav)))
    planner = Planner(track, algorithm=args.algorithm)
    return nav, track, planner.plan_route(
        args.start, args.goal, tuple(args.via or ()), algorithm=args.algorithm
    )


def cmd_plan(args: argparse.Namespace) -> int:
    _nav, _track, route = _plan(args)
    payload = route.as_dict()
    _printer(args)(payload, json.dumps(payload, indent=2))
    return EXIT_OK


def cmd_tags(args: argparse.Namespace) -> int:
    """Stationary camera/OLED diagnostic; never construct a Robot or I2C bus."""
    from .vision.apriltag import (
        NoDisplay,
        OpenCvAprilTagDetector,
        OpenCvCamera,
        YahboomOledDisplay,
        scan_stationary,
    )

    if (
        not math.isfinite(args.duration) or args.duration <= 0
        or not math.isfinite(args.notice_seconds) or args.notice_seconds <= 0
        or args.max_frames is not None and args.max_frames < 1
    ):
        raise ValueError("duration, notice time and max frames must be positive")
    detector = OpenCvAprilTagDetector()
    display = (
        YahboomOledDisplay(
            driver_path=args.oled_driver,
            normal_lines=tuple(args.restore_line or ("Waferbot", "Ready")),
            bus=args.oled_bus,
            address=args.oled_address,
        )
        if args.display == "yahboom"
        else NoDisplay()
    )
    camera = OpenCvCamera(args.camera_index)
    count = scan_stationary(
        camera,
        detector,
        display,
        duration_s=args.duration,
        notice_s=args.notice_seconds,
        max_frames=args.max_frames,
        csv_path=args.log_csv,
    )
    print(f"Tag scan complete: {count} new detection(s), no robot motion")
    return EXIT_OK


def cmd_oled_test(args: argparse.Namespace) -> int:
    """Show the normal OLED screen without touching camera or motors."""
    from .vision.apriltag import YahboomOledDisplay

    if not math.isfinite(args.duration) or args.duration <= 0:
        raise ValueError("duration must be positive")
    display = YahboomOledDisplay(
        driver_path=args.oled_driver,
        normal_lines=("Waferbot", "Ready"),
        bus=args.oled_bus,
        address=args.oled_address,
    )
    try:
        time.sleep(args.duration)
    finally:
        display.close()
    print("OLED displayed: Waferbot / Ready (no camera or robot motion)")
    return EXIT_OK


def cmd_execute(args: argparse.Namespace) -> int:
    # Reject a mock-only localizer before any bus, ownership, or motion work.
    if getattr(args, "physical", False):
        mode = (getattr(args, "localizer", "auto") or "auto").lower()
        if mode in {"scripted", "mock"}:
            raise WaferbotError(
                "--localizer scripted is mock-only; physical execution must use "
                "manual or real marker localization"
            )
    nav, track, route = _plan(args)
    if args.dry_run:
        # A dry run must not construct a bus, open a transport, or emit stops.
        return _execute_dry_run(args, nav, track, route)
    session = _build_session(
        args,
        f"execute {args.start}->{args.goal}",
        motion=True,
        floor_motion=True,
        require_map=True,
        initial_edge=_first_edge(track, route),
    )
    try:
        if session.sensor_script is not None:
            _install_mock_edge_provider(session, _first_edge(track, route))
        # Physical arming consent is the typed confirmation in `_build_session`;
        # mock sessions arm directly.
        session.robot.arm()
        monitor = _arrival_monitor_for(args, session, route)
        executor = RouteExecutor(
            session.robot,
            track,
            nav=nav,
            arrival_monitor=monitor,
            confirm=None if session.mock else _manual_confirm,
            telemetry=session.telemetry,
            sleep=_sleep_for(session),
        )
        result = executor.execute_route(
            route,
            dry_run=False,
            enforce_map_readiness=not session.mock,
            allow_example_map=session.mock,
        )
        payload = result.as_dict()
        _printer(args)(payload, json.dumps(payload, indent=2))
        return EXIT_OK if result.completed else EXIT_REFUSED
    finally:
        session.close()


def _execute_dry_run(args, nav, track, route) -> int:
    """Validate a route without touching hardware or creating a transport."""

    class _DryRunRobot:
        """Minimal stand-in: dry-run execution never commands the robot."""

        config = RobotConfig()
        armed = False
        fault = None

        def __init__(self, config: RobotConfig, clock) -> None:
            self.config = config
            self.clock = clock

        def stop(self) -> None:  # pragma: no cover - dry run never gets here
            raise AssertionError("dry run must not command the robot")

    guard = ProcessGuard(getattr(args, "runtime_dir", None))
    if guard.stop_requested() and not guard.session_is_live():
        # A latched stop does not block planning-only work.
        pass
    telemetry = open_telemetry(None)
    stub = _DryRunRobot(_load_config(args), time.monotonic)
    try:
        executor = RouteExecutor(
            stub,
            track,
            nav=nav,
            arrival_monitor=None,
            telemetry=telemetry,
        )
        result = executor.execute_route(route, dry_run=True, enforce_map_readiness=False)
    finally:
        telemetry.close()
    payload = result.as_dict()
    _printer(args)(payload, json.dumps(payload, indent=2))
    return EXIT_OK if result.completed else EXIT_REFUSED


def _first_edge(track: TrackMap, route) -> EdgeState:
    """Edge the first follow step runs on (defaults to BLACK_LEFT)."""
    for step in route.steps:
        if step.edge.edge_side is not None:
            try:
                return track.edge_state_for(
                    step.edge.edge_side, where=f"{step.source}->{step.destination}"
                )
            except MapError:
                break
    return EdgeState.BLACK_LEFT


class MockEdgeProvider:
    """Mock sensor byte that mirrors a crossing.

    The pattern follows the edge the robot is currently on and flips to the
    opposite edge when the switch manoeuvre starts strafing, which is what a
    real crossing looks like from the sensor's point of view. Mock only.
    """

    def __init__(self, transport, config: RobotConfig, edge: EdgeState) -> None:
        self.transport = transport
        self.config = config
        self.edge = edge
        self._crossing = False

    def __call__(self) -> int:
        if self._is_crossing():
            self.edge = self.edge.opposite
        return edge_byte_for(self.edge, self.config.sensor)

    def _is_crossing(self) -> bool:
        payloads = self.transport.motor_payloads[-4:]
        if len(payloads) != 4:
            self._crossing = False
            return False
        wheels = [
            payload[2] if payload[1] == 0 else -payload[2] for payload in payloads
        ]
        # A lateral crossing makes the two left wheels disagree in the same way
        # as the two right wheels (mecanum strafe or diagonal blend). A forward
        # or differential follow command keeps each pair equal.
        left_delta = wheels[0] - wheels[1]
        right_delta = wheels[3] - wheels[2]
        crossing = (
            left_delta != 0
            and right_delta != 0
            and (left_delta > 0) == (right_delta > 0)
        )
        started = crossing and not self._crossing
        self._crossing = crossing
        return started


def _install_mock_edge_provider(session: Session, edge: EdgeState) -> None:
    provider = MockEdgeProvider(session.transport, session.config, edge)
    session.transport.line_sensor_provider = provider
    session.sensor_script = provider


def _manual_confirm(prompt: str) -> bool:
    return _prompt(prompt + " [yes/no] ").strip().lower() == "yes"


def _arrival_monitor_for(args: argparse.Namespace, session: Session, route):
    """Choose a localization source, never fabricating one for hardware."""
    mode = (getattr(args, "localizer", "auto") or "auto").lower()
    if session.mock:
        if mode in {"auto", "scripted", "mock"}:
            return MockArrivalMonitor(
                polls_before_arrival=args.mock_polls, clock=session.robot.clock
            )
        if mode == "manual":
            return ManualArrivalMonitor(lambda _prompt: True, clock=session.robot.clock)
        return _load_marker_monitor(args, session)

    if mode in {"scripted", "mock"}:
        raise WaferbotError(
            "--localizer scripted is mock-only; physical execution must use "
            "manual or real marker localization"
        )
    if mode == "marker":
        return _load_marker_monitor(args, session)
    if mode in {"auto", "manual"}:
        return ManualArrivalMonitor(_manual_confirm, clock=session.robot.clock)
    raise WaferbotError(f"unsupported localizer {mode!r}")


def _load_marker_monitor(args: argparse.Namespace, session: Session):
    """Load a real marker reader: ``--marker-reader module:attribute``."""
    spec = getattr(args, "marker_reader", None)
    if not spec:
        raise WaferbotError(
            "--localizer marker needs --marker-reader module:attribute returning "
            "an ArrivalMonitor (check(expected_node) -> Localization | None)"
        )
    import importlib

    module_name, _, attribute = spec.partition(":")
    if not module_name or not attribute:
        raise WaferbotError("--marker-reader must look like module:attribute")
    try:
        module = importlib.import_module(module_name)
        factory = getattr(module, attribute)
    except (ImportError, AttributeError) as exc:
        raise WaferbotError(f"cannot load marker reader {spec!r}: {exc}") from exc
    monitor = factory(session=session) if callable(factory) else factory
    for method in ("check", "requires_stop"):
        if not hasattr(monitor, method):
            raise WaferbotError(
                f"marker reader {spec!r} is missing {method}(); it must implement "
                "the ArrivalMonitor contract"
            )
    return monitor


def cmd_stop(args: argparse.Namespace) -> int:
    guard = ProcessGuard(getattr(args, "runtime_dir", None))
    if args.status:
        payload = {
            "runtime_dir": str(guard.runtime_dir),
            "session": guard.session().as_dict() if guard.session() else None,
            "session_live": guard.session_is_live(),
            "owner_lock_available": guard.owner_lock_available(),
            "stop_requested": guard.stop_requested(),
            "stop_request": (
                guard.stop_request().as_dict() if guard.stop_request() else None
            ),
        }
        print(json.dumps(payload, indent=2))
        return EXIT_OK
    if args.clear:
        if not guard.owner_lock_available():
            print(
                "refused: a motion session still holds the bus, so it could "
                "resume and overwrite the stop; stop that session first",
                file=sys.stderr,
            )
            return EXIT_REFUSED
        had = guard.clear_stop_request()
        print("stop latch cleared" if had else "no stop latch was set")
        return EXIT_OK

    request = guard.request_stop(args.reason)
    bus_locked = False
    stop_bytes_written = 0
    error: str | None = None
    if getattr(args, "physical", False):
        config = _load_config(args)
        from .hardware.motor_driver import MotorDriver

        try:
            with guard.bus_guard(timeout_s=args.bus_timeout):
                bus_locked = True
                transport = _open_transport(config)
                try:
                    driver = MotorDriver(
                        transport,
                        address=config.motor.address,
                        invert=config.motor.invert,
                    )
                    driver.stop_all()
                    stop_bytes_written = 4
                finally:
                    closer = getattr(transport, "close", None)
                    if closer is not None:
                        closer()
        except TimeoutError as exc:
            error = f"bus lock timeout: {exc}"
        except I2CError as exc:
            error = str(exc)
    else:
        try:
            with guard.bus_guard(timeout_s=args.bus_timeout):
                bus_locked = True
        except TimeoutError as exc:
            error = f"bus lock timeout: {exc}"
    print(
        json.dumps(
            {
                "stop_requested": True,
                "request": request.as_dict(),
                "bus_lock_acquired": bus_locked,
                "stop_bytes_written": stop_bytes_written,
                "error": error,
                "note": (
                    "The running session checks this latch before every motor "
                    "command and stops. With --physical this command also writes "
                    "the four stop blocks itself, so an absent or dead owner still "
                    "leaves the controller stopped. Motion stays refused until "
                    "`waferbot stop --clear` is run, and clearing is refused while "
                    "a motion session still owns the bus."
                ),
            },
            indent=2,
        )
    )
    if not bus_locked or error is not None:
        return EXIT_REFUSED
    if getattr(args, "physical", False) and stop_bytes_written != 4:
        return EXIT_REFUSED
    return EXIT_OK


# ---------------------------------------------------------------------------
# argument parsing
# ---------------------------------------------------------------------------


def _add_common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--mock",
        dest="physical",
        action="store_false",
        help="use the in-memory mock (default)",
    )
    parser.add_argument(
        "--physical",
        dest="physical",
        action="store_true",
        help="drive real hardware (prompts for confirmation)",
    )
    parser.set_defaults(physical=False)
    parser.add_argument("--config", help="robot configuration JSON")
    parser.add_argument("--nav-config", dest="nav_config", help="navigation config JSON")
    parser.add_argument("--map", help="track map JSON (default maps/example_track.json)")
    parser.add_argument("--runtime-dir", dest="runtime_dir", help="state directory")
    parser.add_argument("--log-csv", dest="log_csv", help="telemetry CSV path")
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    parser.add_argument(
        "--ack-verified-config",
        dest="ack_verified_config",
        action="store_true",
        help=(
            "acknowledge that the wheel/sensor mapping in the config is correct "
            "even if it is not marked verified (floor operation only)"
        ),
    )
    parser.add_argument(
        "--obstacle-distance-mm",
        dest="obstacle_distance_mm",
        type=int,
        default=None,
        help=(
            "physical sessions only: enable the ultrasonic obstacle monitor and "
            "emergency-stop below this distance (disabled by default)"
        ),
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="waferbot",
        description="MicroAlchemy physical robot control (Yahboom Raspbot V2)",
    )
    parser.add_argument("--version", action="version", version=f"waferbot {__version__}")
    subparsers = parser.add_subparsers(dest="command", required=True)

    sensors = subparsers.add_parser("sensors", help="print raw/normalised sensor values")
    _add_common(sensors)
    sensors.add_argument("--samples", type=int, default=5)
    sensors.add_argument("--interval", type=float, default=0.1)
    sensors.set_defaults(func=cmd_sensors)

    ir_strafe = subparsers.add_parser(
        "ir-strafe", help="strafe right and log timed line-sensor readings"
    )
    _add_common(ir_strafe)
    ir_strafe.add_argument("--speed-pwm", type=int, default=5)
    ir_strafe.add_argument("--sample-rate-hz", type=float, default=100.0)
    ir_strafe.add_argument("--duration", type=float, default=2.0)
    ir_strafe.add_argument("--output-csv", required=True)
    ir_strafe.set_defaults(func=cmd_ir_strafe)

    edge_sweep = subparsers.add_parser(
        "edge-angle-sweep", help="constant-angle BLACK_LEFT to BLACK_RIGHT IR experiment"
    )
    _add_common(edge_sweep)
    edge_sweep.add_argument("--angles", default="10:80:10")
    edge_sweep.add_argument("--attempts", type=int, default=3)
    edge_sweep.add_argument("--speed-pwm", type=int, default=5)
    edge_sweep.add_argument("--sample-rate-hz", default="max")
    edge_sweep.add_argument("--duration", type=float, default=3.0)
    edge_sweep.add_argument("--confirm-ms", type=float, default=50.0)
    edge_sweep.add_argument("--output-dir", required=True)
    edge_sweep.set_defaults(func=cmd_edge_angle_sweep)

    tags = subparsers.add_parser(
        "tags", help="stationary tag36h11 camera/OLED diagnostic (no motor control)"
    )
    tags.add_argument("--camera-index", type=int, default=0)
    tags.add_argument("--duration", type=float, default=10.0)
    tags.add_argument("--notice-seconds", type=float, default=1.5)
    tags.add_argument("--max-frames", type=int, default=None)
    tags.add_argument("--display", choices=("yahboom", "none"), default="yahboom")
    tags.add_argument(
        "--oled-driver",
        default=None,
        help="optional external Yahboom OLED driver; default is packaged driver",
    )
    tags.add_argument("--oled-bus", type=int, default=1)
    tags.add_argument("--oled-address", type=lambda value: int(value, 0), default=0x3C)
    tags.add_argument(
        "--restore-line", action="append", default=None,
        help="normal OLED line to restore (pass twice; default: Waferbot / Ready)",
    )
    tags.add_argument("--log-csv", help="append detected tag IDs to this CSV")
    tags.set_defaults(func=cmd_tags)

    oled_test = subparsers.add_parser(
        "oled-test", help="show Waferbot / Ready on the OLED only (no camera or motors)"
    )
    oled_test.add_argument("--duration", type=float, default=3.0)
    oled_test.add_argument("--oled-bus", type=int, default=1)
    oled_test.add_argument("--oled-address", type=lambda value: int(value, 0), default=0x3C)
    oled_test.add_argument("--oled-driver", default=None)
    oled_test.set_defaults(func=cmd_oled_test)

    motor_test = subparsers.add_parser(
        "motor-test", help="run each wheel individually (wheels lifted)"
    )
    _add_common(motor_test)
    motor_test.add_argument("--wheel", type=int, default=None, help="single wheel id 0..3")
    motor_test.add_argument("--speed", type=int, default=40)
    motor_test.add_argument("--duration", type=float, default=1.0)
    motor_test.add_argument(
        "--no-directions", dest="directions", action="store_false", default=True
    )
    motor_test.set_defaults(func=cmd_motor_test)

    calibrate = subparsers.add_parser("calibrate", help="calibration utilities")
    _add_common(calibrate)
    calibrate.add_argument(
        "calibration", choices=["wheels", "sensors", "crossing", "speed"]
    )
    calibrate.add_argument("--wheel", type=int, default=None)
    calibrate.add_argument("--speed", type=int, default=40)
    calibrate.add_argument("--duration", type=float, default=1.0)
    calibrate.add_argument("--no-directions", dest="directions", action="store_false", default=True)
    calibrate.add_argument("--samples", type=int, default=5)
    calibrate.add_argument("--angles", default="10,15,20,25")
    calibrate.add_argument("--attempts", type=int, default=3)
    calibrate.add_argument("--from-edge", dest="from_edge", default="black-left")
    calibrate.add_argument("--to-edge", dest="to_edge", default="black-right")
    calibrate.add_argument("--location", default="B2")
    calibrate.add_argument("--csv", default="crossing_sweep.csv")
    calibrate.add_argument("--counts", type=float, default=None)
    calibrate.add_argument("--distance-m", dest="distance_m", type=float, default=None)
    calibrate.add_argument("--duration-s", dest="duration_s", type=float, default=None)
    calibrate.add_argument("--instructions", action="store_true")
    calibrate.set_defaults(func=cmd_calibrate)

    follow = subparsers.add_parser("follow", help="follow one tape edge")
    _add_common(follow)
    follow.add_argument("--edge", default="black-left")
    follow.add_argument("--duration", type=float, default=10.0)
    follow.add_argument("--iterations", type=int, default=None)
    follow.add_argument("--speed", type=int, default=None)
    follow.add_argument("--rate-hz", dest="rate_hz", type=float, default=None)
    follow.add_argument("--control-log-csv", default=None,
                        help="dedicated per-update PID diagnostic CSV")
    follow.set_defaults(func=cmd_follow)

    switch = subparsers.add_parser("switch", help="switch to the opposite tape edge")
    _add_common(switch)
    switch.add_argument("--from-edge", dest="from_edge", default="black-left")
    switch.add_argument("--to-edge", dest="to_edge", required=True)
    switch.add_argument("--location", default="B2")
    switch.add_argument("--route-id", dest="route_id", default="adhoc")
    switch.add_argument("--destination", default=None)
    switch.add_argument("--authorize", action="store_true")
    switch.set_defaults(func=cmd_switch)

    map_cmd = subparsers.add_parser("map", help="show/validate the track map")
    _add_common(map_cmd)
    map_cmd.add_argument("--require-physical", dest="require_physical", action="store_true")
    map_cmd.set_defaults(func=cmd_map)

    for name, func in (("plan", cmd_plan), ("execute", cmd_execute)):
        sub = subparsers.add_parser(name, help=f"{name} a route")
        _add_common(sub)
        sub.add_argument("--start", required=True)
        sub.add_argument("--goal", required=True)
        sub.add_argument("--via", action="append", default=[])
        sub.add_argument("--algorithm", choices=["dijkstra", "astar"], default="dijkstra")
        sub.set_defaults(func=func)
    execute = subparsers.choices["execute"]
    execute.add_argument("--dry-run", dest="dry_run", action="store_true")
    execute.add_argument(
        "--localizer",
        choices=["auto", "manual", "marker", "scripted"],
        default="auto",
        help=(
            "localization source: auto/manual (operator confirmation with the "
            "wheels stopped) or marker (needs --marker-reader). 'scripted' is "
            "mock-only and refused with --physical."
        ),
    )
    execute.add_argument(
        "--marker-reader",
        dest="marker_reader",
        default=None,
        help="module:attribute returning an ArrivalMonitor (for --localizer marker)",
    )
    execute.add_argument("--mock-polls", dest="mock_polls", type=int, default=2)
    execute.add_argument("--speed", type=int, default=None)

    stop = subparsers.add_parser("stop", help="stop a running session (process-safe)")
    stop.add_argument("--reason", default="operator stop")
    stop.add_argument("--clear", action="store_true", help="clear the stop latch")
    stop.add_argument("--status", action="store_true", help="show session/stop state")
    stop.add_argument("--runtime-dir", dest="runtime_dir", default=None)
    stop.add_argument(
        "--physical",
        dest="physical",
        action="store_true",
        help="also write real stop blocks over I2C (needs the controller)",
    )
    stop.add_argument("--config", default=None, help="robot configuration JSON")
    stop.add_argument(
        "--bus-timeout", dest="bus_timeout", type=float, default=2.0,
        help="seconds to wait for an in-flight bus command to finish",
    )
    stop.set_defaults(func=cmd_stop, physical=False, json=False)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except (
        ConfigError,
        MapError,
        PlanningError,
        ExecutionError,
        AuthorizationError,
        LocalizationError,
        SafetyError,
        I2CError,
        SensorError,
        WaferbotError,
    ) as exc:
        print(f"waferbot: {exc}", file=sys.stderr)
        return EXIT_REFUSED
    except KeyboardInterrupt:
        print("waferbot: interrupted", file=sys.stderr)
        return EXIT_REFUSED
    except (ValueError, OSError) as exc:
        print(f"waferbot: {exc}", file=sys.stderr)
        return EXIT_USAGE


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())


__all__ = ["build_parser", "main"]
