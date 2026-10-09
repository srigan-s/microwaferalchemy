"""Deterministic simulation helpers (no ROS, no hardware).

The tape model here is a *synthetic test bench*: it converts commanded wheel
counts into chassis motion and computed sensor readings so the real
``EdgeFollower`` can be exercised in a closed loop. Its geometry and speed
constants are explicit, labelled assumptions, never measurements.
"""

from .tape_model import (
    ClosedLoopMetrics,
    ClosedLoopRun,
    TapeModelConfig,
    TapeModelTransport,
    Pose,
    run_closed_loop,
)

__all__ = [
    "ClosedLoopMetrics",
    "ClosedLoopRun",
    "Pose",
    "TapeModelConfig",
    "TapeModelTransport",
    "run_closed_loop",
]

