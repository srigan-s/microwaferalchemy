"""Sensing and control: edge detection, edge following, edge switching."""

from .edge import (
    BoundaryEstimate,
    EdgeDetection,
    EdgeDetector,
    EdgeState,
    classify_middle,
    detect_edge,
    edge_byte_for,
    estimate_boundary,
)
from .follower import (
    EdgeFollower,
    FollowResult,
    FollowStopReason,
    wheel_command,
)
from .switching import (
    EdgeSwitcher,
    SwitchAuthorization,
    SwitchPhase,
    SwitchResult,
)

__all__ = [
    "EdgeDetection",
    "BoundaryEstimate",
    "EdgeDetector",
    "EdgeFollower",
    "EdgeState",
    "EdgeSwitcher",
    "FollowResult",
    "FollowStopReason",
    "SwitchAuthorization",
    "SwitchPhase",
    "SwitchResult",
    "classify_middle",
    "detect_edge",
    "edge_byte_for",
    "estimate_boundary",
    "wheel_command",
]
