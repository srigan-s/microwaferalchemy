"""Minimum crossing-angle geometry for edge switching.

Requested calculation::

    minimum_angle = atan((tape_width + 2 * safety_margin)
                         / available_crossing_distance)

That bound only says "this angle produces the lateral displacement". It does not
prove that the four-channel sensor can resolve the transition: sensor spacing,
sensor detection width, sampling rate, switching speed, and available clearance
all matter, so :func:`evaluate_crossing` reports each of those constraints
separately and flags which ones fail. Treat a positive evaluation as a necessary
condition, never as a guarantee.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .errors import ConfigError
from .navconfig import GeometryConfig


@dataclass(frozen=True)
class CrossingInputs:
    """Measured inputs, all in metres except the rate."""

    tape_width_m: float
    sensor_spacing_m: float
    sensor_detection_width_m: float
    available_crossing_distance_m: float
    safety_margin_m: float
    robot_width_m: float
    switching_speed_mps: float
    sampling_rate_hz: float
    lateral_clearance_m: float = 0.0
    required_confirmations: int = 3

    def __post_init__(self) -> None:
        for name in (
            "tape_width_m",
            "sensor_spacing_m",
            "sensor_detection_width_m",
            "available_crossing_distance_m",
            "switching_speed_mps",
            "sampling_rate_hz",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ConfigError(f"{name} must be a number, got {value!r}")
            if not math.isfinite(value) or value <= 0:
                raise ConfigError(f"{name} must be finite and positive, got {value!r}")
        for name in ("safety_margin_m", "robot_width_m", "lateral_clearance_m"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ConfigError(f"{name} must be a number, got {value!r}")
            if not math.isfinite(value) or value < 0:
                raise ConfigError(f"{name} must be finite and >= 0, got {value!r}")
        if (
            isinstance(self.required_confirmations, bool)
            or not isinstance(self.required_confirmations, int)
            or self.required_confirmations < 1
        ):
            raise ConfigError("required_confirmations must be a positive integer")

    @classmethod
    def from_config(cls, geometry: GeometryConfig) -> "CrossingInputs":
        geometry.require_measurements("crossing geometry")
        return cls(
            tape_width_m=geometry.tape_width_m,
            sensor_spacing_m=geometry.sensor_spacing_m,
            sensor_detection_width_m=geometry.sensor_detection_width_m,
            available_crossing_distance_m=geometry.available_crossing_distance_m,
            safety_margin_m=geometry.safety_margin_m,
            robot_width_m=geometry.robot_width_m,
            switching_speed_mps=geometry.switching_speed_mps,
            sampling_rate_hz=geometry.sampling_rate_hz,
            lateral_clearance_m=geometry.lateral_clearance_m,
            required_confirmations=geometry.required_confirmations,
        )

    @classmethod
    def from_measured_command(
        cls,
        geometry: GeometryConfig,
        *,
        speed_counts: int,
        counts_to_mps: float,
        rate_hz: float,
    ) -> "CrossingInputs":
        """Build inputs from the *actual* commanded speed and polling rate.

        The measured geometry supplies lengths; the commanded counts, the
        measured m/s-per-count factor, and the controller's real polling rate
        supply the dynamics, so the observability check is not evaluated against
        unrelated configuration numbers.
        """
        geometry.require_measurements("crossing geometry")
        if (
            isinstance(speed_counts, bool)
            or not isinstance(speed_counts, int)
            or speed_counts < 1
        ):
            raise ConfigError("speed_counts must be a positive integer")
        if not math.isfinite(counts_to_mps) or counts_to_mps <= 0:
            raise ConfigError("counts_to_mps must be finite and positive")
        if not math.isfinite(rate_hz) or rate_hz <= 0:
            raise ConfigError("rate_hz must be finite and positive")
        return cls(
            tape_width_m=geometry.tape_width_m,
            sensor_spacing_m=geometry.sensor_spacing_m,
            sensor_detection_width_m=geometry.sensor_detection_width_m,
            available_crossing_distance_m=geometry.available_crossing_distance_m,
            safety_margin_m=geometry.safety_margin_m,
            robot_width_m=geometry.robot_width_m,
            switching_speed_mps=speed_counts * counts_to_mps,
            sampling_rate_hz=rate_hz,
            lateral_clearance_m=geometry.lateral_clearance_m,
            required_confirmations=geometry.required_confirmations,
        )


@dataclass(frozen=True)
class CrossingEvaluation:
    """Every constraint the requested angle has to satisfy."""

    minimum_angle_deg: float
    requested_angle_deg: float
    lateral_requirement_m: float
    path_length_m: float
    forward_advance_m: float
    sample_distance_m: float
    lateral_speed_mps: float
    observability_required_m: float
    footprint_margin_m: float
    clearance_margin_m: float
    robot_clearance_m: float
    angle_ok: bool
    footprint_ok: bool
    clearance_ok: bool
    robot_clearance_ok: bool
    sampling_ok: bool
    observability_ok: bool
    lateral_clearance_ok: bool
    warnings: tuple[str, ...]

    @property
    def unambiguous_geometry(self) -> bool:
        return (
            self.angle_ok
            and self.footprint_ok
            and self.clearance_ok
            and self.robot_clearance_ok
            and self.sampling_ok
            and self.observability_ok
            and self.lateral_clearance_ok
        )

    def as_dict(self) -> dict[str, object]:
        return {
            "minimum_angle_deg": self.minimum_angle_deg,
            "requested_angle_deg": self.requested_angle_deg,
            "lateral_requirement_m": self.lateral_requirement_m,
            "path_length_m": self.path_length_m,
            "forward_advance_m": self.forward_advance_m,
            "sample_distance_m": self.sample_distance_m,
            "lateral_speed_mps": self.lateral_speed_mps,
            "observability_required_m": self.observability_required_m,
            "footprint_margin_m": self.footprint_margin_m,
            "clearance_margin_m": self.clearance_margin_m,
            "robot_clearance_m": self.robot_clearance_m,
            "angle_ok": self.angle_ok,
            "footprint_ok": self.footprint_ok,
            "clearance_ok": self.clearance_ok,
            "robot_clearance_ok": self.robot_clearance_ok,
            "sampling_ok": self.sampling_ok,
            "observability_ok": self.observability_ok,
            "lateral_clearance_ok": self.lateral_clearance_ok,
            "unambiguous_geometry": self.unambiguous_geometry,
            "warnings": list(self.warnings),
        }


CAVEAT = (
    "the minimum angle only guarantees the requested lateral displacement; "
    "sensor spacing, detection footprint, sampling rate, and clearing distance "
    "must also pass before the transition is unambiguous"
)


def lateral_requirement_m(inputs: CrossingInputs) -> float:
    """Lateral displacement needed to reach the opposite edge, with margins."""
    return inputs.tape_width_m + 2 * inputs.safety_margin_m


def minimum_crossing_angle_deg(inputs: CrossingInputs) -> float:
    """The requested geometric bound, in degrees."""
    return math.degrees(
        math.atan(
            lateral_requirement_m(inputs) / inputs.available_crossing_distance_m
        )
    )


def required_lateral_travel_m(inputs: CrossingInputs) -> float:
    return lateral_requirement_m(inputs)


def evaluate_crossing(
    inputs: CrossingInputs, requested_angle_deg: float
) -> CrossingEvaluation:
    """Evaluate ``requested_angle_deg`` against all known constraints."""
    if not math.isfinite(requested_angle_deg) or not 0 < requested_angle_deg < 90:
        raise ConfigError(
            f"requested crossing angle must be finite and in (0, 90), got "
            f"{requested_angle_deg!r}"
        )
    lateral = lateral_requirement_m(inputs)
    minimum_angle = minimum_crossing_angle_deg(inputs)
    angle_rad = math.radians(requested_angle_deg)
    path_length = lateral / math.sin(angle_rad)
    forward_advance = lateral / math.tan(angle_rad)
    sample_distance = inputs.switching_speed_mps / inputs.sampling_rate_hz

    footprint_resolution = (
        inputs.sensor_detection_width_m + inputs.sensor_spacing_m
    )
    footprint_margin = lateral - footprint_resolution
    clearance_margin = inputs.available_crossing_distance_m - forward_advance
    # The chassis must have room to sit across the tape: that is a *lateral*
    # requirement, compared against measured lateral clearance (never against
    # the longitudinal crossing distance).
    robot_clearance = inputs.tape_width_m + inputs.robot_width_m
    lateral_speed = inputs.switching_speed_mps * math.sin(angle_rad)
    sample_lateral = lateral_speed / inputs.sampling_rate_hz
    observability_required = (
        inputs.sensor_detection_width_m
        + inputs.required_confirmations * sample_lateral
    )

    angle_ok = requested_angle_deg >= minimum_angle - 1e-9
    footprint_ok = footprint_margin >= 0
    clearance_ok = clearance_margin >= 0
    robot_clearance_ok = inputs.lateral_clearance_m >= robot_clearance
    sampling_ok = sample_distance <= footprint_resolution
    observability_ok = lateral >= observability_required
    lateral_clearance_ok = inputs.lateral_clearance_m >= lateral

    warnings: list[str] = [CAVEAT]
    if not angle_ok:
        warnings.append(
            f"requested {requested_angle_deg:.2f} deg is below the geometric "
            f"minimum {minimum_angle:.2f} deg"
        )
    if not footprint_ok:
        warnings.append(
            "lateral requirement "
            f"{lateral:.3f} m is smaller than one sensor pitch plus detection "
            f"width ({footprint_resolution:.3f} m); the opposite channel may "
            "never clearly see the transition"
        )
    if not clearance_ok:
        warnings.append(
            f"the manoeuvre needs {forward_advance:.3f} m of forward travel but "
            f"only {inputs.available_crossing_distance_m:.3f} m is available"
        )
    if not robot_clearance_ok:
        warnings.append(
            f"chassis needs {robot_clearance:.3f} m of measured lateral "
            f"clearance; only {inputs.lateral_clearance_m:.3f} m was measured"
        )
    if not sampling_ok:
        warnings.append(
            f"the robot travels {sample_distance:.4f} m between samples, more "
            f"than the {footprint_resolution:.4f} m sensor resolution"
        )
    if not observability_ok:
        warnings.append(
            f"at {lateral_speed:.4f} m/s lateral speed and "
            f"{inputs.sampling_rate_hz:.1f} Hz sampling, "
            f"{observability_required:.4f} m of lateral travel is needed for "
            f"{inputs.required_confirmations} confirmations but only "
            f"{lateral:.4f} m is available"
        )
    if not lateral_clearance_ok:
        warnings.append(
            f"the crossing needs {lateral:.3f} m of measured lateral clearance; "
            f"only {inputs.lateral_clearance_m:.3f} m was recorded"
        )

    return CrossingEvaluation(
        minimum_angle_deg=minimum_angle,
        requested_angle_deg=float(requested_angle_deg),
        lateral_requirement_m=lateral,
        path_length_m=path_length,
        forward_advance_m=forward_advance,
        sample_distance_m=sample_distance,
        lateral_speed_mps=lateral_speed,
        observability_required_m=observability_required,
        footprint_margin_m=footprint_margin,
        clearance_margin_m=clearance_margin,
        robot_clearance_m=robot_clearance,
        angle_ok=angle_ok,
        footprint_ok=footprint_ok,
        clearance_ok=clearance_ok,
        robot_clearance_ok=robot_clearance_ok,
        sampling_ok=sampling_ok,
        observability_ok=observability_ok,
        lateral_clearance_ok=lateral_clearance_ok,
        warnings=tuple(warnings),
    )


__all__ = [
    "CAVEAT",
    "CrossingEvaluation",
    "CrossingInputs",
    "evaluate_crossing",
    "lateral_requirement_m",
    "minimum_crossing_angle_deg",
    "required_lateral_travel_m",
]
