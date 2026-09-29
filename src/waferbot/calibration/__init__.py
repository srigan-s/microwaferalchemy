"""Calibration utilities: wheel order/polarity, sensors, crossing angles, speed."""

from .crossing import CrossingSweep, CrossingSweepEntry, CrossingSweepReport
from .sensors import (
    CHANNEL_LABELS,
    SensorCalibrationReport,
    SensorSample,
    calibrate_channels,
    format_samples,
    read_samples,
)
from .speed import (
    SpeedCalibrationReport,
    SpeedSample,
    compute_counts_to_mps,
    speed_calibration_instructions,
)
from .wheels import WheelCalibrationReport, WheelCalibrator, WheelProbe

__all__ = [
    "CHANNEL_LABELS",
    "CrossingSweep",
    "CrossingSweepEntry",
    "CrossingSweepReport",
    "SensorCalibrationReport",
    "SensorSample",
    "SpeedCalibrationReport",
    "SpeedSample",
    "WheelCalibrationReport",
    "WheelCalibrator",
    "WheelProbe",
    "calibrate_channels",
    "compute_counts_to_mps",
    "format_samples",
    "read_samples",
    "speed_calibration_instructions",
]

