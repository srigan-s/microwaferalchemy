"""Bounded right-strafe experiment for measuring line-sensor read timing.

The measured rate is the host's completed I2C read rate, not an assertion
about the sensor board's internal optical update rate.
"""

from __future__ import annotations

import csv
import math
import time
from collections.abc import Callable
from pathlib import Path


CSV_FIELDS = (
    "phase", "sample", "read_start_ns", "read_end_ns", "elapsed_s",
    "interval_ms", "read_latency_ms", "raw_byte_hex", "raw_s1", "raw_s2",
    "raw_s3", "raw_s4", "black_s1", "black_s2", "black_s3", "black_s4",
    "black_mask",
)


def run_ir_strafe(
    robot,
    output_csv: str | Path,
    *,
    speed: int,
    rate_hz: float,
    duration_s: float,
    stop_requested: Callable[[], bool] = lambda: False,
    clock_ns: Callable[[], int] = time.monotonic_ns,
    sleep: Callable[[float], None] = time.sleep,
) -> dict[str, object]:
    """Read one stationary baseline, then strafe right and sample until bounded.

    The motor command is refreshed independently of the requested read rate so
    low sampling rates cannot trip the robot's motion watchdog. The robot is
    stopped in ``finally`` even when a read, CSV write, or stop check fails.
    """
    if isinstance(speed, bool) or not isinstance(speed, int) or not 1 <= speed <= robot.config.motor.max_speed:
        raise ValueError(f"speed must be 1..{robot.config.motor.max_speed} PWM")
    if isinstance(rate_hz, bool) or not isinstance(rate_hz, (int, float)) or not math.isfinite(rate_hz) or not 1 <= rate_hz <= 500:
        raise ValueError("rate_hz must be finite and between 1 and 500")
    if isinstance(duration_s, bool) or not isinstance(duration_s, (int, float)) or not math.isfinite(duration_s) or not 0 < duration_s <= 10:
        raise ValueError("duration_s must be finite and between 0 and 10 seconds")
    if not robot.armed:
        raise ValueError("robot must be armed before the IR strafe experiment")

    path = Path(output_csv)
    path.parent.mkdir(parents=True, exist_ok=True)
    period_ns = round(1_000_000_000 / rate_hz)
    refresh_ns = round(min(0.1, robot.config.safety.motion_timeout_s / 3) * 1_000_000_000)
    samples = 0
    transitions = 0
    missed_deadlines = 0
    first_read_ns: int | None = None
    last_read_ns: int | None = None
    last_mask: int | None = None
    stop_reason = "DURATION"
    started_ns: int | None = None
    try:
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
            writer.writeheader()

            def record(phase: str, index: int, run_start_ns: int | None, previous_ns: int | None):
                read_start = clock_ns()
                reading = robot.read_line_sensors()
                read_end = clock_ns()
                row = {
                    "phase": phase,
                    "sample": index,
                    "read_start_ns": read_start,
                    "read_end_ns": read_end,
                    "elapsed_s": "" if run_start_ns is None else f"{(read_end - run_start_ns) / 1e9:.6f}",
                    "interval_ms": "" if previous_ns is None else f"{(read_end - previous_ns) / 1e6:.3f}",
                    "read_latency_ms": f"{(read_end - read_start) / 1e6:.3f}",
                    "raw_byte_hex": f"0x{reading.raw_byte:02X}",
                    "black_mask": f"0x{reading.black_mask:X}",
                }
                for channel in range(4):
                    row[f"raw_s{channel + 1}"] = reading.raw[channel]
                    row[f"black_s{channel + 1}"] = reading.normalized[channel]
                writer.writerow(row)
                handle.flush()
                return read_end, reading.black_mask

            if stop_requested():
                stop_reason = "STOP_REQUESTED"
            else:
                record("baseline", 0, None, None)
                if stop_requested():
                    stop_reason = "STOP_REQUESTED"
                else:
                    robot.strafe_right(speed)
                    started_ns = clock_ns()
                    end_ns = started_ns + round(duration_s * 1_000_000_000)
                    next_read_ns = started_ns
                    next_refresh_ns = started_ns + refresh_ns
                    while True:
                        now = clock_ns()
                        if stop_requested():
                            stop_reason = "STOP_REQUESTED"
                            break
                        if robot.fault is not None:
                            stop_reason = "FAULT"
                            break
                        if now >= end_ns:
                            break
                        if now >= next_refresh_ns:
                            robot.strafe_right(speed)
                            next_refresh_ns = clock_ns() + refresh_ns
                            continue
                        if now >= next_read_ns:
                            completed_ns, mask = record("moving", samples + 1, started_ns, last_read_ns)
                            samples += 1
                            if first_read_ns is None:
                                first_read_ns = completed_ns
                            if last_mask is not None and mask != last_mask:
                                transitions += 1
                            last_mask = mask
                            last_read_ns = completed_ns
                            next_read_ns += period_ns
                            if completed_ns >= next_read_ns:
                                skipped = (completed_ns - next_read_ns) // period_ns + 1
                                missed_deadlines += skipped
                                next_read_ns += skipped * period_ns
                            continue
                        sleep(max(0.0, (min(next_read_ns, next_refresh_ns, end_ns) - now) / 1e9))
    finally:
        robot.stop()

    elapsed_s = 0.0 if started_ns is None else max(0.0, (clock_ns() - started_ns) / 1e9)
    observed_hz = None
    if samples > 1 and first_read_ns is not None and last_read_ns is not None and last_read_ns > first_read_ns:
        observed_hz = (samples - 1) * 1e9 / (last_read_ns - first_read_ns)
    return {
        "output_csv": str(path),
        "stop_reason": stop_reason,
        "speed_pwm": speed,
        "requested_rate_hz": rate_hz,
        "observed_rate_hz": observed_hz,
        "duration_s": duration_s,
        "elapsed_s": elapsed_s,
        "moving_samples": samples,
        "pattern_transitions": transitions,
        "missed_read_deadlines": missed_deadlines,
        "rate_meaning": "completed host I2C reads per second; not sensor internal update rate",
    }
