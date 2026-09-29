"""Typed error hierarchy for the waferbot hardware layer.

I2C failures are never swallowed. Every transport failure surfaces as an
``I2CError`` (or a subclass) so callers can react to real bus problems
instead of silently driving on stale assumptions.
"""


class WaferbotError(Exception):
    """Base class for every error raised by this package."""


class ConfigError(WaferbotError):
    """Raised when configuration is missing, malformed, or out of range."""


class I2CError(WaferbotError):
    """Raised when an I2C transfer fails.

    The original ``OSError`` (or ``ImportError`` for a missing smbus2) is kept
    as ``__cause__``.
    """


class MotorCommandError(I2CError):
    """A motor command failed part-way through a multi-wheel write.

    The driver attempts a best-effort stop of all four wheels before this is
    raised; the originating :class:`I2CError` is chained as ``__cause__``.
    """


class SensorError(WaferbotError):
    """Raised when a sensor frame is malformed or cannot be decoded."""


class SafetyError(WaferbotError):
    """Raised when a motion request violates the safety contract."""


class SpeedLimitError(SafetyError, ValueError):
    """Raised when a requested speed is outside the configured bound."""


class AuthorizationError(SafetyError):
    """Raised when an edge switch is attempted without a valid authorization."""


class SwitchError(WaferbotError):
    """Raised when an edge-switching manoeuvre cannot be completed safely."""


class MapError(ConfigError):
    """Raised when a track map is malformed, inconsistent, or not actionable."""


class PlanningError(WaferbotError):
    """Raised when no valid route satisfies the request."""


class LocalizationError(WaferbotError):
    """Raised when the robot cannot establish or confirm its location."""


class ExecutionError(WaferbotError):
    """Raised when a route cannot be executed as planned."""
