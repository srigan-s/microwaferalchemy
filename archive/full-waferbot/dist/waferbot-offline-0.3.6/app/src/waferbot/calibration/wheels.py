"""Wheel order, sign, and direction calibration.

Nothing here can detect motion on its own: this chassis has no encoders in the
vendored protocol, so the operator is the sensor. Each probe runs exactly one
motor for a bounded time, stops it, and asks the operator what happened. Until
every probe is confirmed "yes", the report marks the mapping as uncertain, which
is what the CLI prints before any floor operation.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from ..errors import SafetyError


@dataclass(frozen=True)
class WheelProbe:
    index: int
    label: str
    motor_id: int
    speed: int
    duration_s: float
    answer: str
    notes: str = ""

    @property
    def confirmed(self) -> bool:
        return self.answer == "yes"

    def as_dict(self) -> dict[str, Any]:
        return {
            "index": self.index,
            "label": self.label,
            "motor_id": self.motor_id,
            "speed": self.speed,
            "duration_s": self.duration_s,
            "answer": self.answer,
            "confirmed": self.confirmed,
            "notes": self.notes,
        }


@dataclass
class WheelCalibrationReport:
    probes: list[WheelProbe] = field(default_factory=list)
    direction_checks: dict[str, str] = field(default_factory=dict)
    suggested_invert: list[int] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    @property
    def confirmed(self) -> bool:
        """Every wheel probe *and* every requested direction check must pass."""
        if not self.all_wheels_tested:
            # A partial run cannot verify the chassis mapping.
            return False
        if not all(probe.confirmed for probe in self.probes):
            return False
        if self.direction_checks:
            return all(
                answer == "yes" for answer in self.direction_checks.values()
            )
        return True

    @property
    def all_wheels_tested(self) -> bool:
        return {probe.motor_id for probe in self.probes} == {0, 1, 2, 3}

    def as_dict(self) -> dict[str, Any]:
        return {
            "confirmed": self.confirmed,
            "uncertain_until_confirmed": not self.confirmed,
            "probes": [probe.as_dict() for probe in self.probes],
            "direction_checks": self.direction_checks,
            "suggested_invert": self.suggested_invert,
            "notes": self.notes,
        }


class WheelCalibrator:
    """Drives one wheel at a time and records the operator's observation."""

    def __init__(
        self,
        robot,
        *,
        ask: Callable[[str], str],
        sleep: Callable[[float], None] = time.sleep,
        telemetry=None,
    ) -> None:
        self.robot = robot
        self.ask = ask
        self.sleep = sleep
        self.telemetry = telemetry

    def probe(
        self,
        motor_id: int,
        *,
        speed: int = 40,
        duration_s: float = 1.0,
        label: str | None = None,
        expect: str = "forward",
    ) -> WheelProbe:
        if not self.robot.armed:
            raise SafetyError("wheel calibration requires an armed robot")
        labels = self.robot.config.motor.wheel_labels
        resolved_label = label or labels[motor_id]
        speeds = [0, 0, 0, 0]
        speeds[motor_id] = speed
        # Hold the single-wheel command through the robot's refresh loop so a
        # probe longer than the watchdog timeout cannot trip MOTION_TIMEOUT.
        self.robot.run_command(
            lambda: self.robot.drive_wheels(
                speeds, context=f"calibrate-wheel-{motor_id}"
            ),
            duration_s,
        )
        answer = self.ask(
            f"Wheel id {motor_id} ({resolved_label}) just ran at {speed} for "
            f"{duration_s:.1f}s. Did it turn {expect}? [yes/no/unknown]"
        )
        answer = str(answer).strip().lower()
        if answer not in {"yes", "no", "unknown", "skip"}:
            answer = "unknown"
        return WheelProbe(
            index=motor_id,
            label=resolved_label,
            motor_id=motor_id,
            speed=speed,
            duration_s=duration_s,
            answer=answer,
            notes="" if answer == "yes" else "operator did not confirm",
        )

    def run(
        self,
        *,
        speed: int = 40,
        duration_s: float = 1.0,
        wheels: list[int] | None = None,
        directions: bool = True,
    ) -> WheelCalibrationReport:
        report = WheelCalibrationReport()
        report.notes.append(
            "Wheels must be off the ground and the chassis supported. The "
            "mapping stays uncertain until every probe is confirmed."
        )
        selected = list(range(4)) if wheels is None else list(wheels)
        for motor_id in selected:
            if motor_id not in range(4):
                raise ValueError(f"wheel id {motor_id} outside 0..3")
            probe = self.probe(
                motor_id, speed=speed, duration_s=duration_s, expect="forward"
            )
            report.probes.append(probe)
            if probe.answer == "no":
                report.suggested_invert.append(motor_id)

        if directions:
            report.direction_checks = self.check_directions(
                speed=speed, duration_s=duration_s
            )
        if report.suggested_invert:
            report.notes.append(
                "Wheels "
                + ", ".join(str(wheel) for wheel in report.suggested_invert)
                + " reported the wrong direction: set motor.invert for those ids."
            )
        return report

    def check_directions(
        self, *, speed: int = 40, duration_s: float = 1.0
    ) -> dict[str, str]:
        """Verify forward/backward, strafing, and rotation as whole-chassis moves."""
        checks: dict[str, str] = {}
        moves = (
            ("forward", lambda: self.robot.forward(speed), "drive forward"),
            ("backward", lambda: self.robot.backward(speed), "drive backward"),
            ("strafe_left", lambda: self.robot.strafe_left(speed), "strafe left"),
            ("strafe_right", lambda: self.robot.strafe_right(speed), "strafe right"),
            ("rotate_left", lambda: self.robot.rotate_left(speed), "rotate left"),
            ("rotate_right", lambda: self.robot.rotate_right(speed), "rotate right"),
        )
        for name, action, description in moves:
            # Every movement goes through the robot's bounded, refreshed hold so
            # it cannot outlive the motion watchdog, and it always finally-stops.
            self.robot.run_command(action, duration_s, context=f"direction-{name}")
            answer = str(
                self.ask(
                    f"Whole-chassis check: did the robot {description}? "
                    "[yes/no/skip]"
                )
            ).strip().lower()
            checks[name] = answer if answer in {"yes", "no", "skip"} else "unknown"
        return checks


__all__ = ["WheelCalibrationReport", "WheelCalibrator", "WheelProbe"]
