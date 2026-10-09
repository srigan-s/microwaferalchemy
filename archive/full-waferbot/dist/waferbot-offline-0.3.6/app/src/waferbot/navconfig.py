"""Navigation configuration: follower, switcher, geometry, and execution.

Kept separate from :mod:`waferbot.config` because these are controller
parameters, not hardware wiring. Values are validated strictly, and the
geometry values are explicitly marked ``measured`` so nothing pretends that a
counts-per-second guess is a physical speed.
"""

from __future__ import annotations

import json
import math
from collections.abc import Mapping
from dataclasses import dataclass, field, fields, replace
from pathlib import Path
from typing import Any

from .errors import ConfigError


def _check_number(value, name: str, *, minimum: float | None = None, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ConfigError(f"{name} must be a number, got {value!r}")
    if not math.isfinite(value):
        raise ConfigError(f"{name} must be finite, got {value!r}")
    if positive and value <= 0:
        raise ConfigError(f"{name} must be positive, got {value!r}")
    if minimum is not None and value < minimum:
        raise ConfigError(f"{name} must be >= {minimum}, got {value!r}")
    return value


def _check_int(value, name: str, *, minimum: int | None = None, positive=False):
    """PWM counts, rates, and sample counts must be true integers."""
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(f"{name} must be an integer count, got {value!r}")
    if positive and value <= 0:
        raise ConfigError(f"{name} must be positive, got {value!r}")
    if minimum is not None and value < minimum:
        raise ConfigError(f"{name} must be >= {minimum}, got {value!r}")
    return value


@dataclass(frozen=True)
class FollowConfig:
    """Edge-following controller parameters (PWM counts, no m/s pretence).

    Explicit gains use measured millimetres and seconds. The old per-pitch
    fields remain as read-only migration inputs when explicit gains are absent.
    """

    rate_hz: float = 100.0
    ir_polling_hz: float = 300.0
    sensor_positions_mm: tuple[float, float, float, float] = (-31.25, -3.25, 3.25, 31.25)
    reference_mm: float = 0.0
    controller_enabled: bool = False
    base_speed: int = 40
    #: Legacy per-central-gap proportional and derivative gains.
    kp: float = 12.0
    kd: float = 0.2
    #: Preferred dimensionally explicit gains. Legacy kp/kd are interpreted
    #: per central (6.5 mm) sensor gap when these are omitted.
    kp_pwm_per_mm: float | None = None
    ki_pwm_per_mm_s: float = 0.0
    kd_pwm_s_per_mm: float | None = None
    integral_limit_pwm: float = 0.0
    velocity_filter_tau_s: float = 0.05
    max_correction_slew_pwm_per_s: float | None = None
    max_correction: int = 30
    #: Legacy per-cycle slew limit; converted using the historic 20 Hz rate.
    max_correction_delta: int = 8
    #: Minimum forward-speed fraction at maximum steering demand. Reducing the
    #: forward component gives the differential term more authority in bends.
    min_curve_speed_factor: float = 0.45
    #: Correction-demand fraction at which curve slowdown begins.
    curve_slowdown_start: float = 0.20
    #: Deprecated; velocity_filter_tau_s controls derivative filtering.
    derivative_filter_alpha: float = 0.5
    correction_mode: str = "differential"
    lateral_weight: float = 0.5
    #: Confident consecutive estimates required before the first forward motion.
    acquisition_samples: int = 3
    acquisition_timeout_s: float = 3.0
    acquisition_confidence: float = 0.6
    #: Estimates below this confidence count as ambiguous observations.
    ambiguous_confidence: float = 0.6
    #: How long rejected (stale/replayed/future) readings may persist.
    max_rejected_s: float = 0.3
    junction_speed_factor: float = 0.5
    junction_channels: int = 3
    edge_loss_samples: int = 3
    recovery_enabled: bool = True
    recovery_speed: int = 25
    recovery_reverse_s: float = 0.15
    recovery_search_s: float = 1.0
    recovery_attempts: int = 2
    stable_samples: int = 3
    sensor_max_age_s: float = 0.25
    max_duration_s: float = 120.0
    #: How long ambiguous (both-black/both-white) or wrong-edge readings may
    #: persist before the controller treats the line as lost.
    max_ambiguous_s: float = 1.5
    max_wrong_edge_s: float = 1.0
    #: Localization evidence required to accept an arrival.
    arrival_confirmations: int = 1
    localization_max_age_s: float = 5.0
    localization_min_confidence: float = 0.5

    def validate(self) -> "FollowConfig":
        _check_number(self.rate_hz, "follow.rate_hz", positive=True)
        _check_number(self.ir_polling_hz, "follow.ir_polling_hz", positive=True)
        if len(self.sensor_positions_mm) != 4:
            raise ConfigError("follow.sensor_positions_mm needs four positions")
        positions = tuple(_check_number(p, "follow.sensor_positions_mm") for p in self.sensor_positions_mm)
        if any(b <= a for a, b in zip(positions, positions[1:])):
            raise ConfigError("follow.sensor_positions_mm must increase left to right")
        _check_number(self.reference_mm, "follow.reference_mm")
        if not isinstance(self.controller_enabled, bool):
            raise ConfigError("follow.controller_enabled must be a bool")
        for name in ("kp_pwm_per_mm", "kd_pwm_s_per_mm", "max_correction_slew_pwm_per_s"):
            value = getattr(self, name)
            if value is not None:
                _check_number(value, f"follow.{name}", minimum=0)
        _check_number(self.ki_pwm_per_mm_s, "follow.ki_pwm_per_mm_s", minimum=0)
        _check_number(self.integral_limit_pwm, "follow.integral_limit_pwm", minimum=0)
        _check_number(self.velocity_filter_tau_s, "follow.velocity_filter_tau_s", positive=True)
        if self.ki_pwm_per_mm_s > 0 and self.integral_limit_pwm <= 0:
            raise ConfigError("positive Ki requires integral_limit_pwm > 0")
        _check_int(self.base_speed, "follow.base_speed", positive=True)
        _check_number(self.kp, "follow.kp", minimum=0)
        _check_number(self.kd, "follow.kd", minimum=0)
        _check_int(self.max_correction, "follow.max_correction", minimum=0)
        _check_int(
            self.max_correction_delta, "follow.max_correction_delta", minimum=1
        )
        for name in ("min_curve_speed_factor", "curve_slowdown_start"):
            _check_number(getattr(self, name), f"follow.{name}", minimum=0)
            if getattr(self, name) > 1:
                raise ConfigError(f"follow.{name} must be <= 1")
        _check_number(
            self.derivative_filter_alpha,
            "follow.derivative_filter_alpha",
            minimum=0,
        )
        if self.derivative_filter_alpha > 1:
            raise ConfigError("follow.derivative_filter_alpha must be <= 1")
        _check_int(self.acquisition_samples, "follow.acquisition_samples", minimum=1)
        _check_number(
            self.acquisition_timeout_s, "follow.acquisition_timeout_s", positive=True
        )
        for name in ("acquisition_confidence", "ambiguous_confidence"):
            _check_number(getattr(self, name), f"follow.{name}", minimum=0)
            if getattr(self, name) > 1:
                raise ConfigError(f"follow.{name} must be <= 1")
        _check_number(self.max_rejected_s, "follow.max_rejected_s", positive=True)
        if self.correction_mode not in {"differential", "lateral", "blended"}:
            raise ConfigError(
                "follow.correction_mode must be differential, lateral, or blended"
            )
        _check_number(self.lateral_weight, "follow.lateral_weight", minimum=0)
        if self.lateral_weight > 1:
            raise ConfigError("follow.lateral_weight must be <= 1")
        _check_number(
            self.junction_speed_factor, "follow.junction_speed_factor", positive=True
        )
        if self.junction_speed_factor > 1:
            raise ConfigError("follow.junction_speed_factor must be <= 1")
        _check_int(self.junction_channels, "follow.junction_channels", minimum=1)
        if self.junction_channels > 4:
            raise ConfigError("follow.junction_channels must be <= 4")
        _check_int(self.edge_loss_samples, "follow.edge_loss_samples", minimum=1)
        _check_int(self.recovery_speed, "follow.recovery_speed", minimum=0)
        _check_number(
            self.recovery_reverse_s, "follow.recovery_reverse_s", minimum=0
        )
        _check_number(self.recovery_search_s, "follow.recovery_search_s", minimum=0)
        _check_int(self.recovery_attempts, "follow.recovery_attempts", minimum=0)
        _check_int(self.stable_samples, "follow.stable_samples", minimum=1)
        _check_number(
            self.sensor_max_age_s, "follow.sensor_max_age_s", positive=True
        )
        _check_number(self.max_duration_s, "follow.max_duration_s", positive=True)
        _check_number(self.max_ambiguous_s, "follow.max_ambiguous_s", positive=True)
        _check_number(self.max_wrong_edge_s, "follow.max_wrong_edge_s", positive=True)
        _check_number(
            self.arrival_confirmations, "follow.arrival_confirmations", minimum=1
        )
        if int(self.arrival_confirmations) != self.arrival_confirmations:
            raise ConfigError("follow.arrival_confirmations must be an integer")
        _check_number(
            self.localization_max_age_s,
            "follow.localization_max_age_s",
            positive=True,
        )
        _check_number(
            self.localization_min_confidence,
            "follow.localization_min_confidence",
            minimum=0,
        )
        if self.localization_min_confidence > 1:
            raise ConfigError("follow.localization_min_confidence must be <= 1")
        if not isinstance(self.recovery_enabled, bool):
            raise ConfigError("follow.recovery_enabled must be a bool")
        return self


@dataclass(frozen=True)
class SwitchConfig:
    """Edge-switching manoeuvre parameters."""

    mode: str = "lateral"
    speed: int = 35
    crossing_angle_deg: float | None = None
    approach_speed: int = 30
    reduce_speed_factor: float = 0.6
    confirm_samples: int = 3
    stable_samples: int = 3
    resume_samples: int = 2
    sensor_max_age_s: float = 0.25
    establish_timeout_s: float = 3.0
    switch_timeout_s: float = 3.0
    search_timeout_s: float = 2.0
    verify_timeout_s: float = 2.0
    max_travel_s: float = 4.0
    authorization_max_age_s: float = 30.0
    require_destination_confirmation: bool = True
    destination_confirmations: int = 1
    localization_max_age_s: float = 5.0
    localization_min_confidence: float = 0.5
    #: Measured, conservative straight-line travel budget for a physical switch.
    max_travel_m: float = 0.0
    #: Sensor polling rate actually used during the crossing.
    poll_rate_hz: float = 20.0

    def validate(self) -> "SwitchConfig":
        if self.mode not in {"lateral", "diagonal"}:
            raise ConfigError("switch.mode must be lateral or diagonal")
        _check_int(self.speed, "switch.speed", positive=True)
        _check_int(self.approach_speed, "switch.approach_speed", positive=True)
        if self.mode == "diagonal":
            if self.crossing_angle_deg is None:
                raise ConfigError(
                    "switch.crossing_angle_deg is required for diagonal switching"
                )
            _check_number(
                self.crossing_angle_deg, "switch.crossing_angle_deg", positive=True
            )
            if not 0 < self.crossing_angle_deg < 90:
                raise ConfigError("switch.crossing_angle_deg must be in (0, 90)")
        elif self.crossing_angle_deg is not None:
            _check_number(
                self.crossing_angle_deg, "switch.crossing_angle_deg", positive=True
            )
            if not 0 < self.crossing_angle_deg < 90:
                raise ConfigError("switch.crossing_angle_deg must be in (0, 90)")
        _check_number(
            self.reduce_speed_factor, "switch.reduce_speed_factor", positive=True
        )
        if self.reduce_speed_factor > 1:
            raise ConfigError("switch.reduce_speed_factor must be <= 1")
        _check_int(self.confirm_samples, "switch.confirm_samples", minimum=1)
        _check_int(self.stable_samples, "switch.stable_samples", minimum=1)
        _check_int(self.resume_samples, "switch.resume_samples", minimum=0)
        for name in (
            "sensor_max_age_s",
            "establish_timeout_s",
            "switch_timeout_s",
            "search_timeout_s",
            "verify_timeout_s",
            "max_travel_s",
            "authorization_max_age_s",
        ):
            _check_number(getattr(self, name), f"switch.{name}", positive=True)
        if not isinstance(self.require_destination_confirmation, bool):
            raise ConfigError("switch.require_destination_confirmation must be a bool")
        _check_number(
            self.destination_confirmations,
            "switch.destination_confirmations",
            minimum=1,
        )
        if int(self.destination_confirmations) != self.destination_confirmations:
            raise ConfigError("switch.destination_confirmations must be an integer")
        _check_number(
            self.localization_max_age_s,
            "switch.localization_max_age_s",
            positive=True,
        )
        _check_number(
            self.localization_min_confidence,
            "switch.localization_min_confidence",
            minimum=0,
        )
        if self.localization_min_confidence > 1:
            raise ConfigError("switch.localization_min_confidence must be <= 1")
        _check_number(self.max_travel_m, "switch.max_travel_m", minimum=0)
        _check_number(self.poll_rate_hz, "switch.poll_rate_hz", positive=True)
        return self


@dataclass(frozen=True)
class GeometryConfig:
    """Physically measured chassis and tape geometry.

    Every length is in metres and comes from a tape measure, not from a guess.
    ``measured`` stays ``False`` until an operator has recorded them; while it is
    false, diagonal switching and any graph distance/speed enforcement refuse to
    run instead of pretending an uncalibrated number is a distance.
    """

    tape_width_m: float = 0.0
    sensor_spacing_m: float = 0.0
    sensor_detection_width_m: float = 0.0
    available_crossing_distance_m: float = 0.0
    safety_margin_m: float = 0.0
    robot_width_m: float = 0.0
    switching_speed_mps: float = 0.0
    sampling_rate_hz: float = 0.0
    #: Measured free space to the side of the track (lateral clearance).
    lateral_clearance_m: float = 0.0
    #: How many consecutive fresh confirmations the controller needs.
    required_confirmations: int = 3
    measured: bool = False

    def validate(self) -> "GeometryConfig":
        for name in (
            "tape_width_m",
            "sensor_spacing_m",
            "sensor_detection_width_m",
            "available_crossing_distance_m",
            "safety_margin_m",
            "robot_width_m",
            "switching_speed_mps",
            "sampling_rate_hz",
            "lateral_clearance_m",
        ):
            _check_number(getattr(self, name), f"geometry.{name}", minimum=0)
        _check_int(
            self.required_confirmations, "geometry.required_confirmations", minimum=1
        )
        if not isinstance(self.measured, bool):
            raise ConfigError("geometry.measured must be a bool")
        if self.measured:
            for name in (
                "tape_width_m",
                "sensor_spacing_m",
                "sensor_detection_width_m",
                "available_crossing_distance_m",
                "switching_speed_mps",
                "sampling_rate_hz",
            ):
                if getattr(self, name) <= 0:
                    raise ConfigError(
                        f"geometry.{name} must be > 0 once geometry.measured is true"
                    )
        return self

    def require_measurements(self, purpose: str) -> None:
        if not self.measured:
            raise ConfigError(
                f"{purpose} needs measured geometry; run "
                "`waferbot calibrate crossing --record ...` (or set "
                "geometry.measured=true) with real tape measurements first"
            )


@dataclass(frozen=True)
class ExecutionConfig:
    """Route-execution bounds."""

    max_edge_travel_s: float = 60.0
    max_route_duration_s: float = 900.0
    turn_speed: int = 30
    dock_speed: int = 20
    allow_example_map: bool = False
    require_start_confirmation: bool = True
    require_arrival_confirmation: bool = True
    arrival_confirmations: int = 1
    localization_max_age_s: float = 5.0
    localization_min_confidence: float = 0.5

    def validate(self) -> "ExecutionConfig":
        for name in ("max_edge_travel_s", "max_route_duration_s"):
            _check_number(getattr(self, name), f"execution.{name}", positive=True)
        _check_int(self.turn_speed, "execution.turn_speed", positive=True)
        _check_int(self.dock_speed, "execution.dock_speed", positive=True)
        for name in (
            "allow_example_map",
            "require_start_confirmation",
            "require_arrival_confirmation",
        ):
            if not isinstance(getattr(self, name), bool):
                raise ConfigError(f"execution.{name} must be a bool")
        _check_number(
            self.arrival_confirmations, "execution.arrival_confirmations", minimum=1
        )
        if int(self.arrival_confirmations) != self.arrival_confirmations:
            raise ConfigError("execution.arrival_confirmations must be an integer")
        _check_number(
            self.localization_max_age_s,
            "execution.localization_max_age_s",
            positive=True,
        )
        _check_number(
            self.localization_min_confidence,
            "execution.localization_min_confidence",
            minimum=0,
        )
        if self.localization_min_confidence > 1:
            raise ConfigError("execution.localization_min_confidence must be <= 1")
        return self


@dataclass(frozen=True)
class NavConfig:
    follow: FollowConfig = field(default_factory=FollowConfig)
    switch: SwitchConfig = field(default_factory=SwitchConfig)
    geometry: GeometryConfig = field(default_factory=GeometryConfig)
    execution: ExecutionConfig = field(default_factory=ExecutionConfig)
    telemetry_csv: str | None = None
    telemetry_every_n: int = 1
    map_path: str | None = None
    #: Optional measured conversion. ``None`` means "no m/s claims".
    counts_to_mps: float | None = None
    counts_to_mps_note: str = ""

    def validate(self) -> "NavConfig":
        self.follow.validate()
        self.switch.validate()
        self.geometry.validate()
        self.execution.validate()
        _check_number(self.telemetry_every_n, "telemetry_every_n", minimum=1)
        for name in ("telemetry_csv", "map_path"):
            value = getattr(self, name)
            if value is not None and not isinstance(value, str):
                raise ConfigError(f"{name} must be a string or null")
        if self.counts_to_mps is not None:
            _check_number(self.counts_to_mps, "counts_to_mps", positive=True)
            if not isinstance(self.counts_to_mps_note, str):
                raise ConfigError("counts_to_mps_note must be a string")
        return self

    @property
    def has_speed_calibration(self) -> bool:
        return self.counts_to_mps is not None

    @classmethod
    def from_dict(cls, data: Mapping[str, Any] | None) -> "NavConfig":
        if data is None:
            return cls().validate()
        if not isinstance(data, Mapping):
            raise ConfigError("nav configuration must be a mapping")
        data = {key: value for key, value in data.items() if not str(key).startswith("_")}
        known = {f.name for f in fields(cls)}
        unknown = set(data) - known
        if unknown:
            raise ConfigError(f"unknown navigation configuration keys: {sorted(unknown)}")
        values: dict[str, Any] = {}
        for section, section_cls in (
            ("follow", FollowConfig),
            ("switch", SwitchConfig),
            ("geometry", GeometryConfig),
            ("execution", ExecutionConfig),
        ):
            raw = data.get(section)
            if raw is None:
                continue
            if not isinstance(raw, Mapping):
                raise ConfigError(f"nav.{section} must be a mapping")
            section_known = {f.name for f in fields(section_cls)}
            section_unknown = set(raw) - section_known
            if section_unknown:
                raise ConfigError(
                    f"unknown keys in nav.{section}: {sorted(section_unknown)}"
                )
            section_data = dict(raw)
            if section == "follow" and "sensor_positions_mm" in section_data:
                section_data["sensor_positions_mm"] = tuple(section_data["sensor_positions_mm"])
            values[section] = section_cls(**section_data)
        for key in (
            "telemetry_csv",
            "telemetry_every_n",
            "map_path",
            "counts_to_mps",
            "counts_to_mps_note",
        ):
            if key in data:
                values[key] = data[key]
        return cls(**values).validate()

    def to_dict(self) -> dict[str, Any]:
        def section(obj) -> dict[str, Any]:
            return {f.name: getattr(obj, f.name) for f in fields(obj)}

        return {
            "follow": section(self.follow),
            "switch": section(self.switch),
            "geometry": section(self.geometry),
            "execution": section(self.execution),
            "telemetry_csv": self.telemetry_csv,
            "telemetry_every_n": self.telemetry_every_n,
            "map_path": self.map_path,
            "counts_to_mps": self.counts_to_mps,
            "counts_to_mps_note": self.counts_to_mps_note,
        }

    @classmethod
    def load_json(cls, path: str | Path) -> "NavConfig":
        path = Path(path)
        try:
            raw = json.loads(path.read_text(encoding="utf-8"))
        except FileNotFoundError as exc:
            raise ConfigError(f"navigation configuration not found: {path}") from exc
        except json.JSONDecodeError as exc:
            raise ConfigError(
                f"navigation configuration {path} is not valid JSON: {exc}"
            ) from exc
        return cls.from_dict(raw)

    def save_json(self, path: str | Path) -> None:
        Path(path).write_text(
            json.dumps(self.to_dict(), indent=2) + "\n", encoding="utf-8"
        )

    def with_follow(self, **kwargs) -> "NavConfig":
        return replace(self, follow=replace(self.follow, **kwargs)).validate()

    def with_mode(self, mode: str, crossing_angle_deg: float | None = None) -> "NavConfig":
        return replace(
            self,
            switch=replace(
                self.switch, mode=mode, crossing_angle_deg=crossing_angle_deg
            ),
        ).validate()


__all__ = [
    "ExecutionConfig",
    "FollowConfig",
    "GeometryConfig",
    "NavConfig",
    "SwitchConfig",
]
