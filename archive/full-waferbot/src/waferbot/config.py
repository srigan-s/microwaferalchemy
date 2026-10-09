"""Configuration primitives for the physical robot.

Values are plain dataclasses with strict validation and an optional JSON
round-trip so an operator can keep two boards (or two tape layouts) apart
without editing code. Nothing here touches hardware.

Defaults marked "unverified" are documented in ``HARDWARE_IMPLEMENTATION.md``
and must be confirmed with the phase 2 diagnostics before real motion.
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

from .errors import ConfigError
from .protocol import (
    I2C_ADDRESS_DEFAULT,
    I2C_BUS_DEFAULT,
    MOTOR_SPEED_MAX,
    REG_LINE_SENSOR,
)

#: Vendor-derived channel order. See HARDWARE_IMPLEMENTATION.md for evidence.
DEFAULT_SENSOR_BITS: tuple[int, int, int, int] = (2, 3, 1, 0)


@dataclass(frozen=True)
class MotorConfig:
    """Motor driver configuration.

    ``max_speed`` is the software bound applied by :class:`waferbot.robot.Robot`
    for normal navigation; ``MOTOR_SPEED_MAX`` (255) is the protocol bound.
    """

    address: int = I2C_ADDRESS_DEFAULT
    bus: int = I2C_BUS_DEFAULT
    max_speed: int = 100
    #: Wheel id -> physical position label, in motor-id order.
    #: The L1/L2, R1/R2 front/rear split is NOT established by vendored source;
    #: it only matters for diagonal moves, which phase 1 does not expose.
    wheel_labels: tuple[str, str, str, str] = (
        "front_left",
        "rear_left",
        "front_right",
        "rear_right",
    )
    #: Per-wheel polarity correction produced by `waferbot calibrate wheels`.
    #: ``True`` inverts that wheel's commanded direction.
    invert: tuple[bool, bool, bool, bool] = (False, False, False, False)
    #: True once an operator has confirmed wheel order and directions on the
    #: real chassis (`waferbot calibrate wheels`). Floor operation requires it.
    verified: bool = False

    def validate(self) -> None:
        if not isinstance(self.address, int) or not 0x03 <= self.address <= 0x77:
            raise ConfigError(f"motor.address must be a 7-bit I2C address, got {self.address!r}")
        if not isinstance(self.bus, int) or self.bus < 0:
            raise ConfigError(f"motor.bus must be a non-negative int, got {self.bus!r}")
        if not isinstance(self.max_speed, int) or isinstance(self.max_speed, bool):
            raise ConfigError(f"motor.max_speed must be an int, got {self.max_speed!r}")
        if not 0 < self.max_speed <= MOTOR_SPEED_MAX:
            raise ConfigError(
                f"motor.max_speed must be in 1..{MOTOR_SPEED_MAX}, got {self.max_speed}"
            )
        if len(self.wheel_labels) != 4:
            raise ConfigError("motor.wheel_labels must list exactly four labels")
        if len(self.invert) != 4 or not all(
            isinstance(flag, bool) for flag in self.invert
        ):
            raise ConfigError("motor.invert must be four booleans, one per wheel id")
        if not isinstance(self.verified, bool):
            raise ConfigError("motor.verified must be a boolean")


@dataclass(frozen=True)
class SensorConfig:
    """Four-channel line sensor configuration.

    ``bit_for_channel`` maps physical channels S1..S4 (left to right, as the
    operator sees them from behind the robot) to bit positions in the byte read
    from ``register``. ``black_is_raw_zero`` records the polarity: with the
    vendored firmware and black tape, a detected line reads 0 and white reads 1.
    """

    address: int = I2C_ADDRESS_DEFAULT
    bus: int = I2C_BUS_DEFAULT
    register: int = REG_LINE_SENSOR
    bit_for_channel: tuple[int, int, int, int] = DEFAULT_SENSOR_BITS
    black_is_raw_zero: bool = True
    #: True once an operator has confirmed channel order and polarity on the
    #: real chassis (`waferbot calibrate sensors`). Floor operation requires it.
    verified: bool = False

    def validate(self) -> None:
        if not isinstance(self.address, int) or not 0x03 <= self.address <= 0x77:
            raise ConfigError(f"sensor.address must be a 7-bit I2C address, got {self.address!r}")
        if not isinstance(self.bus, int) or self.bus < 0:
            raise ConfigError(f"sensor.bus must be a non-negative int, got {self.bus!r}")
        if not isinstance(self.register, int) or not 0x00 <= self.register <= 0xFF:
            raise ConfigError(f"sensor.register must be a byte, got {self.register!r}")
        if sorted(self.bit_for_channel) != [0, 1, 2, 3]:
            raise ConfigError(
                "sensor.bit_for_channel must be a permutation of (0, 1, 2, 3), "
                f"got {self.bit_for_channel!r}"
            )
        if not isinstance(self.black_is_raw_zero, bool):
            raise ConfigError(
                f"sensor.black_is_raw_zero must be a bool, got {self.black_is_raw_zero!r}"
            )
        if not isinstance(self.verified, bool):
            raise ConfigError("sensor.verified must be a boolean")


@dataclass(frozen=True)
class SafetyConfig:
    """Motion watchdog and arming policy."""

    #: A motion command is valid for this long without a refresh.
    motion_timeout_s: float = 0.5
    #: Fastest loop the watchdog can be enforced at, for documentation only.
    watchdog_period_s: float = 0.05

    def validate(self) -> None:
        for name in ("motion_timeout_s", "watchdog_period_s"):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise ConfigError(f"safety.{name} must be a number, got {value!r}")
            if not math.isfinite(value) or value <= 0:
                raise ConfigError(f"safety.{name} must be finite and positive, got {value!r}")
        if self.watchdog_period_s > self.motion_timeout_s:
            raise ConfigError(
                "safety.watchdog_period_s must not exceed safety.motion_timeout_s"
            )


@dataclass(frozen=True)
class RobotConfig:
    """Top-level configuration for :class:`waferbot.robot.Robot`."""

    motor: MotorConfig = field(default_factory=MotorConfig)
    sensor: SensorConfig = field(default_factory=SensorConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)

    def validate(self) -> "RobotConfig":
        self.motor.validate()
        self.sensor.validate()
        self.safety.validate()
        if self.motor.bus != self.sensor.bus:
            # The controller board carries both the motor bridge and the line
            # sensor on one bus. Silently opening two buses would mean the
            # watchdog's stop lock and the sensor lock guard different objects,
            # so a mismatch is rejected instead.
            raise ConfigError(
                "motor.bus and sensor.bus must match "
                f"(got {self.motor.bus} and {self.sensor.bus}); this controller "
                "exposes motors and line sensors on the same I2C bus"
            )
        return self

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "RobotConfig":
        if not isinstance(data, Mapping):
            raise ConfigError(f"configuration must be a mapping, got {type(data).__name__}")
        # Keys starting with "_" are documentation only (for example "_comment").
        data = {key: value for key, value in data.items() if not str(key).startswith("_")}
        known = {"motor", "sensor", "safety"}
        unknown = set(data) - known
        if unknown:
            raise ConfigError(f"unknown configuration sections: {sorted(unknown)}")
        config = cls(
            motor=_section(MotorConfig, data.get("motor"), "motor"),
            sensor=_section(SensorConfig, data.get("sensor"), "sensor"),
            safety=_section(SafetyConfig, data.get("safety"), "safety"),
        )
        return config.validate()

    def to_dict(self) -> dict[str, Any]:
        return {
            "motor": {
                "address": self.motor.address,
                "bus": self.motor.bus,
                "max_speed": self.motor.max_speed,
                "wheel_labels": list(self.motor.wheel_labels),
                "invert": list(self.motor.invert),
                "verified": self.motor.verified,
            },
            "sensor": {
                "address": self.sensor.address,
                "bus": self.sensor.bus,
                "register": self.sensor.register,
                "bit_for_channel": list(self.sensor.bit_for_channel),
                "black_is_raw_zero": self.sensor.black_is_raw_zero,
                "verified": self.sensor.verified,
            },
            "safety": {
                "motion_timeout_s": self.safety.motion_timeout_s,
                "watchdog_period_s": self.safety.watchdog_period_s,
            },
        }

    @classmethod
    def load_json(cls, path: str | Path) -> "RobotConfig":
        path = Path(path)
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise ConfigError(f"configuration file not found: {path}") from exc
        except json.JSONDecodeError as exc:
            raise ConfigError(f"configuration file {path} is not valid JSON: {exc}") from exc
        return cls.from_dict(raw)

    def save_json(self, path: str | Path) -> None:
        Path(path).write_text(
            json.dumps(self.to_dict(), indent=2, sort_keys=False) + "\n",
            encoding="utf-8",
        )

    def with_speed_limit(self, max_speed: int) -> "RobotConfig":
        """Return a copy with a different navigation speed bound."""
        return replace(self, motor=replace(self.motor, max_speed=max_speed)).validate()

    def with_inverted_wheels(self, wheels: Sequence[int]) -> "RobotConfig":
        """Return a copy with the given wheel ids polarity-inverted."""
        flags = list(self.motor.invert)
        for wheel in wheels:
            if wheel not in range(4):
                raise ConfigError(f"wheel id {wheel!r} outside 0..3")
            flags[wheel] = True
        return replace(self, motor=replace(self.motor, invert=tuple(flags))).validate()

    def unverified_components(self) -> list[str]:
        """Hardware assumptions that have not been confirmed on this chassis."""
        pending = []
        if not self.motor.verified:
            pending.append(
                "motor: wheel order/sign mapping (run `waferbot calibrate wheels`)"
            )
        if not self.sensor.verified:
            pending.append(
                "sensor: channel order/polarity (run `waferbot calibrate sensors`)"
            )
        return pending

    def require_verified_hardware(self) -> None:
        """Refuse floor operation until the measured assumptions are confirmed."""
        pending = self.unverified_components()
        if not pending:
            return
        from .errors import SafetyError

        raise SafetyError(
            "floor operation refused: the following hardware assumptions are not "
            "verified in configuration:\n  - "
            + "\n  - ".join(pending)
            + "\nRun the calibrations, set `verified: true` in the robot config, "
            "or acknowledge explicitly with --ack-verified-config."
        )


def _section(cls: type, data: Any, name: str):
    if data is None:
        return cls()
    if not isinstance(data, Mapping):
        raise ConfigError(f"configuration section {name!r} must be a mapping")
    fields = set(cls.__dataclass_fields__)
    unknown = set(data) - fields
    if unknown:
        raise ConfigError(f"unknown keys in {name!r}: {sorted(unknown)}")
    return cls(**dict(data))


__all__ = [
    "DEFAULT_SENSOR_BITS",
    "MotorConfig",
    "RobotConfig",
    "SafetyConfig",
    "SensorConfig",
]
