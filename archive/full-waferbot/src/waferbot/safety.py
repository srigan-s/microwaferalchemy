"""Safety states, faults, arming, and the motion watchdog.

The contract this module enforces (see ``HARDWARE_IMPLEMENTATION.md``):

* Motion is impossible until an operator explicitly arms the robot.
* A latched emergency stop survives until an explicit, confirmed clear, and
  clearing it never re-arms the robot by itself.
* Every motion command carries a deadline. If the control loop stops
  refreshing it, the watchdog stops the wheels.
* Faults are typed. State transitions are explicit, so a future controller can
  read and assert on them without reaching into the driver.

Nothing here imports smbus2 or touches hardware; the stop action is injected.
"""

from __future__ import annotations

import time
from collections.abc import Callable
from dataclasses import dataclass, replace
from enum import Enum

from .errors import SafetyError


class SafetyState(str, Enum):
    """Operating state of the robot, as required by the request."""

    IDLE = "IDLE"
    FOLLOWING = "FOLLOWING"
    SWITCHING = "SWITCHING"
    DOCKING = "DOCKING"
    COMPLETE = "COMPLETE"
    FAULT = "FAULT"
    EMERGENCY_STOP = "EMERGENCY_STOP"


class FaultCode(str, Enum):
    """Fault conditions the robot must be able to report."""

    LINE_LOST = "LINE_LOST"
    SENSOR_FAILURE = "SENSOR_FAILURE"
    SWITCH_TIMEOUT = "SWITCH_TIMEOUT"
    LOCALIZATION_FAILURE = "LOCALIZATION_FAILURE"
    MOTOR_COMMUNICATION_FAILURE = "MOTOR_COMMUNICATION_FAILURE"
    OBSTACLE_DETECTED = "OBSTACLE_DETECTED"
    #: Added by the phase 2 safety corrections: the independent watchdog fired
    #: because the control loop stopped refreshing its motion deadline.
    MOTION_TIMEOUT = "MOTION_TIMEOUT"


@dataclass(frozen=True)
class FaultRecord:
    """A fault with the time it was observed."""

    code: FaultCode
    message: str
    timestamp: float


class SafetyController:
    """Arming, latching faults, and the motion deadline watchdog.

    ``stop_callback`` is invoked to command an all-wheel stop. It is called
    with motors already logically stopped (state is latched first) and any
    exception it raises propagates to the caller after the latch is set, so a
    failed I2C stop is never hidden.
    """

    def __init__(
        self,
        stop_callback: Callable[[], None],
        *,
        clock: Callable[[], float] = time.monotonic,
        motion_timeout_s: float = 0.5,
    ) -> None:
        if motion_timeout_s <= 0:
            raise SafetyError("motion_timeout_s must be positive")
        self._stop_callback = stop_callback
        self._clock = clock
        self._motion_timeout_s = float(motion_timeout_s)
        self._state = SafetyState.IDLE
        self._armed = False
        self._fault: FaultRecord | None = None
        self._emergency_stop_latched = False
        self._deadline: float | None = None

    # -- inspection ---------------------------------------------------------

    @property
    def state(self) -> SafetyState:
        return self._state

    @property
    def armed(self) -> bool:
        return self._armed

    @property
    def fault(self) -> FaultRecord | None:
        return self._fault

    @property
    def emergency_stop_latched(self) -> bool:
        return self._emergency_stop_latched

    @property
    def motion_timeout_s(self) -> float:
        return self._motion_timeout_s

    @property
    def deadline(self) -> float | None:
        return self._deadline

    # -- arming -------------------------------------------------------------

    def arm(self) -> None:
        """Explicitly enable motion. Idempotent once armed."""
        if self._emergency_stop_latched:
            raise SafetyError("cannot arm while an emergency stop is latched")
        if self._fault is not None:
            raise SafetyError(
                f"cannot arm while fault {self._fault.code.value} is active"
            )
        self._armed = True
        if self._state in (SafetyState.FAULT, SafetyState.EMERGENCY_STOP):
            self._state = SafetyState.IDLE

    def disarm(self) -> None:
        """Disable motion without changing any latched fault."""
        self._armed = False
        self._deadline = None
        if self._state not in (SafetyState.FAULT, SafetyState.EMERGENCY_STOP):
            self._state = SafetyState.IDLE

    def require_armed(self) -> None:
        """Raise unless motion is currently permitted."""
        if self._emergency_stop_latched:
            raise SafetyError("motion refused: emergency stop is latched")
        if self._fault is not None:
            raise SafetyError(
                f"motion refused: fault {self._fault.code.value} is active"
            )
        if not self._armed:
            raise SafetyError("motion refused: robot is not armed")

    # -- faults -------------------------------------------------------------

    def set_state(self, state: SafetyState) -> None:
        """Record the controller's operating state."""
        if self._state is SafetyState.EMERGENCY_STOP:
            raise SafetyError("cannot change state while an emergency stop is latched")
        self._state = state

    def raise_fault(self, code: FaultCode, message: str = "") -> FaultRecord:
        """Latch a fault and disable motion. Does not command the motors."""
        record = FaultRecord(code=code, message=message, timestamp=self._clock())
        self._fault = record
        self._armed = False
        self._deadline = None
        self._state = SafetyState.FAULT
        return record

    def note_failure(self, message: str) -> FaultRecord:
        """Record a failure that happened while handling another condition.

        Used when a stop attempt itself fails: the existing fault (or emergency
        stop) is kept, its message is extended, and the robot stays disarmed.
        If no fault was latched yet, a
        :attr:`FaultCode.MOTOR_COMMUNICATION_FAILURE` fault is created, because
        the only failure that reaches here is a failed bus command.
        """
        if self._fault is None:
            return self.raise_fault(FaultCode.MOTOR_COMMUNICATION_FAILURE, message)
        combined = (
            f"{self._fault.message} | {message}" if self._fault.message else message
        )
        self._fault = replace(self._fault, message=combined)
        self._armed = False
        self._deadline = None
        if self._state is not SafetyState.EMERGENCY_STOP:
            self._state = SafetyState.FAULT
        return self._fault

    def clear_fault(self) -> None:
        """Clear a latched fault without arming. Refuses after an e-stop."""
        if self._emergency_stop_latched:
            raise SafetyError(
                "cannot clear faults while an emergency stop is latched"
            )
        if self._fault is None:
            raise SafetyError("no fault is latched")
        self._fault = None
        self._state = SafetyState.IDLE

    def emergency_stop(
        self, code: FaultCode = FaultCode.MOTOR_COMMUNICATION_FAILURE, message: str = ""
    ) -> FaultRecord:
        """Latch an emergency stop and command all-wheel stop.

        The latch is set before the stop is attempted, so even a failing stop
        callback leaves the robot disarmed and refusing motion.
        """
        record = FaultRecord(code=code, message=message, timestamp=self._clock())
        self._fault = record
        self._armed = False
        self._deadline = None
        self._emergency_stop_latched = True
        self._state = SafetyState.EMERGENCY_STOP
        self._stop_callback()
        return record

    def clear_emergency_stop(self, *, confirm: bool = False) -> None:
        """Release a latched emergency stop.

        ``confirm`` must be ``True``: releasing an e-stop is an explicit
        operator decision. The robot stays disarmed afterwards.
        """
        if not self._emergency_stop_latched:
            raise SafetyError("no emergency stop is latched")
        if confirm is not True:
            raise SafetyError("clearing an emergency stop requires confirm=True")
        self._emergency_stop_latched = False
        self._fault = None
        self._armed = False
        self._state = SafetyState.IDLE

    # -- motion deadline watchdog ------------------------------------------

    def note_motion(self, timeout_s: float | None = None) -> None:
        """Record that a motion command was issued and refresh the deadline."""
        window = self._motion_timeout_s if timeout_s is None else float(timeout_s)
        if window <= 0:
            raise SafetyError("motion timeout must be positive")
        self._deadline = self._clock() + window

    def clear_motion(self) -> None:
        """Cancel the pending deadline (for example after a stop)."""
        self._deadline = None

    def deadline_expired(self) -> bool:
        return self._deadline is not None and self._clock() >= self._deadline

    def enforce_deadline(self) -> bool:
        """Stop the wheels if the motion deadline has passed.

        Latches ``MOTION_TIMEOUT`` first, then stops, so a synchronous expiry is
        recorded exactly like an independent-watchdog expiry. Exceptions from the
        stop callback propagate after the fault is latched.
        """
        if not self.deadline_expired():
            return False
        self._deadline = None
        self.raise_fault(
            FaultCode.MOTION_TIMEOUT,
            "motion deadline expired without a refresh from the controller",
        )
        self._stop_callback()
        return True


__all__ = [
    "FaultCode",
    "FaultRecord",
    "SafetyController",
    "SafetyState",
]
