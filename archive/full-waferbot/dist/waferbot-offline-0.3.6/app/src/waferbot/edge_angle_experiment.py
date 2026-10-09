"""Open-loop straight diagonal crossings with high-rate IR transition logging.

This is an experiment, not an authorized navigation edge switch. It observes
BLACK_LEFT -> BLACK_RIGHT at the middle sensor pair and never infers location.
"""

from __future__ import annotations

import csv
import math
import statistics
import time
from collections.abc import Callable
from pathlib import Path

from .sensing.edge import EdgeState, classify_middle


TRACE_FIELDS = (
    "phase", "sample", "requested_angle_deg", "commanded_angle_deg",
    "read_start_ns", "read_end_ns", "elapsed_ms", "interval_ms",
    "read_latency_ms", "raw_byte_hex", "raw_s1", "raw_s2", "raw_s3",
    "raw_s4", "black_s1", "black_s2", "black_s3", "black_s4",
    "edge_state", "edge_changed", "since_last_change_ms",
)
SUMMARY_FIELDS = (
    "angle_deg", "attempt", "commanded_angle_deg", "wheels", "speed_pwm",
    "requested_rate_hz",
    "stop_reason", "source_acquired", "target_confirmed", "switch_time_ms",
    "confirmation_time_ms", "moving_samples", "observed_rate_hz",
    "elapsed_s", "trace_csv", "edge_changes",
)


def straight_wheels(angle_deg: float, speed: int) -> tuple[tuple[int, int, int, int], float]:
    """Closest representable forward/left vector at the configured PWM ceiling.

    The mecanum signs are (f-l, f+l, f+l, f-l). Integer PWM quantizes the
    requested direction; this function reports the realized command angle.
    """
    if not math.isfinite(angle_deg) or not 0 < angle_deg < 90:
        raise ValueError("angle_deg must be finite and between 0 and 90")
    if isinstance(speed, bool) or not isinstance(speed, int) or not 1 <= speed <= 255:
        raise ValueError("speed must be a positive integer PWM ceiling")
    choices = []
    for a in range(-speed, speed + 1):
        for b in range(-speed, speed + 1):
            forward = (a + b) / 2
            left = (b - a) / 2
            if forward <= 0 or left <= 0:
                continue
            realized = math.degrees(math.atan2(left, forward))
            choices.append((abs(realized - angle_deg), -max(abs(a), abs(b)), a, b, realized))
    if not choices:
        raise ValueError("PWM ceiling cannot represent a forward-left diagonal")
    _, _, a, b, realized = min(choices)
    return (a, b, b, a), realized


def run_straight_crossing(
    robot,
    trace_csv: str | Path,
    *,
    angle_deg: float,
    speed: int,
    duration_s: float,
    sample_rate_hz: float | None = None,
    confirm_ms: float = 50.0,
    stop_requested: Callable[[], bool] = lambda: False,
    clock_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, object]:
    if isinstance(speed, bool) or not isinstance(speed, int) or not 1 <= speed <= robot.config.motor.max_speed:
        raise ValueError(f"speed must be 1..{robot.config.motor.max_speed} PWM")
    wheels, realized_angle = straight_wheels(angle_deg, speed)
    if not math.isfinite(duration_s) or not 0 < duration_s <= 10:
        raise ValueError("duration_s must be between 0 and 10 seconds")
    if sample_rate_hz is not None and (not math.isfinite(sample_rate_hz) or not 1 <= sample_rate_hz <= 500):
        raise ValueError("sample_rate_hz must be 1..500 or None for maximum")
    if not math.isfinite(confirm_ms) or not 0 < confirm_ms <= 1000:
        raise ValueError("confirm_ms must be between 0 and 1000")
    if not robot.armed:
        raise ValueError("robot must be armed")

    path = Path(trace_csv)
    path.parent.mkdir(parents=True, exist_ok=True)
    period_ns = None if sample_rate_hz is None else round(1e9 / sample_rate_hz)
    refresh_ns = round(min(0.1, robot.config.safety.motion_timeout_s / 3) * 1e9)
    confirm_ns = round(confirm_ms * 1e6)
    started_ns = None
    first_moving_ns = None
    last_moving_ns = None
    previous_read_ns = None
    previous_edge = None
    last_change_ns = None
    source_last_ns = None
    target_first_ns = None
    target_count = 0
    edge_changes = []
    samples = 0
    source_acquired = False
    target_confirmed = False
    reason = "DURATION"
    try:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=TRACE_FIELDS)
            writer.writeheader()

            def read_row(phase: str, index: int):
                nonlocal previous_read_ns, previous_edge, last_change_ns
                before = clock_ns()
                reading = robot.read_line_sensors()
                after = clock_ns()
                state = classify_middle(reading.normalized)
                changed = previous_edge is not None and state is not previous_edge
                since_change = None if last_change_ns is None else (after - last_change_ns) / 1e6
                if changed:
                    edge_changes.append({
                        "from": previous_edge.value, "to": state.value,
                        "at_ms": None if started_ns is None else (after - started_ns) / 1e6,
                        "since_previous_change_ms": since_change,
                    })
                    last_change_ns = after
                elif last_change_ns is None:
                    last_change_ns = after
                row = {
                    "phase": phase, "sample": index,
                    "requested_angle_deg": angle_deg,
                    "commanded_angle_deg": f"{realized_angle:.3f}",
                    "read_start_ns": before, "read_end_ns": after,
                    "elapsed_ms": "" if started_ns is None else f"{(after-started_ns)/1e6:.3f}",
                    "interval_ms": "" if previous_read_ns is None else f"{(after-previous_read_ns)/1e6:.3f}",
                    "read_latency_ms": f"{(after-before)/1e6:.3f}",
                    "raw_byte_hex": f"0x{reading.raw_byte:02X}",
                    "edge_state": state.value,
                    "edge_changed": int(changed),
                    "since_last_change_ms": "" if since_change is None else f"{since_change:.3f}",
                }
                for channel in range(4):
                    row[f"raw_s{channel+1}"] = reading.raw[channel]
                    row[f"black_s{channel+1}"] = reading.normalized[channel]
                writer.writerow(row)
                previous_read_ns = after
                previous_edge = state
                return after, state

            if stop_requested():
                reason = "STOP_REQUESTED"
            else:
                # Three stationary source-edge readings prevent a run starting
                # from an unknown position. No motion occurs on failed acquire.
                baseline_readings = [read_row("baseline", i) for i in range(3)]
                baseline = [state for _, state in baseline_readings]
                source_acquired = all(state is EdgeState.BLACK_LEFT for state in baseline)
                if not source_acquired:
                    reason = "SOURCE_NOT_ACQUIRED"
                elif stop_requested():
                    reason = "STOP_REQUESTED"
                else:
                    source_last_ns = baseline_readings[-1][0]
                    robot.drive_wheels(wheels, context="edge_angle_experiment")
                    started_ns = clock_ns()
                    end_ns = started_ns + round(duration_s * 1e9)
                    next_refresh_ns = started_ns + refresh_ns
                    next_read_ns = started_ns
                    while True:
                        now = clock_ns()
                        if stop_requested():
                            reason = "STOP_REQUESTED"
                            break
                        if robot.fault is not None:
                            reason = "FAULT"
                            break
                        if now >= end_ns or samples >= 20_000:
                            reason = "DURATION" if now >= end_ns else "SAMPLE_LIMIT"
                            break
                        if now >= next_refresh_ns:
                            robot.drive_wheels(wheels, context="edge_angle_experiment")
                            next_refresh_ns = clock_ns() + refresh_ns
                            continue
                        if period_ns is None or now >= next_read_ns:
                            completed, state = read_row("moving", samples + 1)
                            samples += 1
                            if first_moving_ns is None:
                                first_moving_ns = completed
                            last_moving_ns = completed
                            if state is EdgeState.BLACK_LEFT:
                                source_last_ns = completed
                                target_first_ns = None
                                target_count = 0
                            elif state is EdgeState.BLACK_RIGHT:
                                if target_first_ns is None:
                                    target_first_ns = completed
                                    target_count = 0
                                target_count += 1
                                if (source_last_ns is not None and target_count >= 3
                                        and completed - target_first_ns >= confirm_ns):
                                    target_confirmed = True
                                    reason = "TARGET_CONFIRMED"
                                    break
                            else:
                                target_first_ns = None
                                target_count = 0
                            if period_ns is not None:
                                next_read_ns += period_ns
                                if completed >= next_read_ns:
                                    next_read_ns += ((completed - next_read_ns) // period_ns + 1) * period_ns
                            continue
                        sleep(max(0.0, (min(next_read_ns, next_refresh_ns, end_ns) - now) / 1e9))
    finally:
        robot.stop()

    elapsed_s = None if started_ns is None else max(0.0, (clock_ns() - started_ns) / 1e9)
    observed_hz = None
    if samples > 1 and first_moving_ns is not None and last_moving_ns > first_moving_ns:
        observed_hz = (samples - 1) * 1e9 / (last_moving_ns - first_moving_ns)
    switch_ms = None
    confirm_time_ms = None
    if target_confirmed and source_last_ns is not None and target_first_ns is not None:
        switch_ms = (target_first_ns - source_last_ns) / 1e6
        confirm_time_ms = (last_moving_ns - source_last_ns) / 1e6
    return {
        "angle_deg": angle_deg,
        "commanded_angle_deg": realized_angle,
        "wheels": list(wheels),
        "speed_pwm": speed,
        "requested_rate_hz": "max" if sample_rate_hz is None else sample_rate_hz,
        "stop_reason": reason,
        "source_acquired": source_acquired,
        "target_confirmed": target_confirmed,
        "switch_time_ms": switch_ms,
        "confirmation_time_ms": confirm_time_ms,
        "moving_samples": samples,
        "observed_rate_hz": observed_hz,
        "elapsed_s": elapsed_s,
        "trace_csv": str(path),
        "edge_changes": edge_changes,
    }


def append_summary(path: str | Path, result: dict[str, object], attempt: int) -> None:
    """Persist each completed attempt before the next repositioning prompt."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    new = not path.exists() or path.stat().st_size == 0
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=SUMMARY_FIELDS)
        if new:
            writer.writeheader()
        row = {key: result.get(key) for key in SUMMARY_FIELDS}
        row["attempt"] = attempt
        row["wheels"] = " ".join(map(str, result["wheels"]))
        row["edge_changes"] = len(result["edge_changes"])
        writer.writerow(row)


def minimum_confirmed_angle(results: list[dict[str, object]], attempts: int) -> float | None:
    """Smallest tested angle whose every repeated trial confirmed the target."""
    by_angle: dict[float, list[bool]] = {}
    for result in results:
        by_angle.setdefault(float(result["angle_deg"]), []).append(bool(result["target_confirmed"]))
    eligible = [angle for angle, values in by_angle.items() if len(values) == attempts and all(values)]
    return min(eligible) if eligible else None


def summarize_angles(
    results: list[dict[str, object]], angles: list[float], attempts: int,
) -> list[dict[str, object]]:
    """Summarize evidence without treating a sensor edge as a map arrival."""
    summaries = []
    first_for_vector: dict[tuple[int, ...], float] = {}
    for angle in angles:
        trials = [row for row in results if float(row["angle_deg"]) == angle]
        successful = [row for row in trials if row["target_confirmed"]]
        vector = tuple(straight_wheels(angle, int(trials[0]["speed_pwm"]))[0]) if trials else None
        duplicate_of = first_for_vector.get(vector) if vector is not None else None
        if vector is not None:
            first_for_vector.setdefault(vector, angle)
        times = [float(row["switch_time_ms"]) for row in successful if row["switch_time_ms"] is not None]
        rates = [float(row["observed_rate_hz"]) for row in trials if row["observed_rate_hz"] is not None]
        summaries.append({
            "requested_angle_deg": angle,
            "commanded_angle_deg": None if not trials else trials[0]["commanded_angle_deg"],
            "wheels": None if vector is None else list(vector),
            "attempts_completed": len(trials),
            "target_confirmations": len(successful),
            "success_rate": None if not trials else len(successful) / len(trials),
            "all_repeats_confirmed": len(trials) == attempts and len(successful) == attempts,
            "median_switch_time_ms": None if not times else statistics.median(times),
            "median_observed_rate_hz": None if not rates else statistics.median(rates),
            "duplicate_command_of_deg": duplicate_of,
        })
    return summaries
