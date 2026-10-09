"""High-level robot facade: the interface navigation code calls.

Movement interface (as requested by the project)::

    robot.forward(speed) / robot.backward(speed)
    robot.strafe_left(speed) / robot.strafe_right(speed)
    robot.rotate_left(speed) / robot.rotate_right(speed)
    robot.stop()

plus :meth:`Robot.drive_wheels` for controllers that compute per-wheel values
and :meth:`Robot.run_command` for bounded manoeuvres that must keep refreshing
the watchdog.

Atomicity contract
------------------
Every motion command runs inside one transaction that holds the in-process
:class:`~waferbot.hardware.bus.BusLock` (re-entrant) and the cross-process bus
guard (``flock`` in the runtime directory, re-entrant inside one process so
nested stop paths cannot deadlock). Inside that transaction the robot checks the
external stop latch, the arming state, latched faults and the emergency stop,
refreshes the watchdog deadline, and only then writes the wheels. A stop that
latches while a command is in flight can therefore never be followed by nonzero
wheel writes.

Safety guarantees
-----------------
* Import and construction perform no I2C write.
* Motion requires an explicit :meth:`arm` and is refused while faulted.
* Failures latch typed faults, attempt stops, and re-raise the original error.
* ``disarm()``/``stop()`` stop the wheels; a failed stop stays disarmed and is
  recorded rather than hidden.
* An independent watchdog thread latches :attr:`FaultCode.MOTION_TIMEOUT` and
  stops the wheels even if the controller never calls :meth:`tick` again, and
  :meth:`tick` never cancels a live deadline.
* Wheel counts are integers; fractional and boolean values are rejected.

These are software guards only. Nothing here can cut motor power, so a process
kill, a power loss, or a wedged I2C bus still needs a physical power switch.
"""

from __future__ import annotations

import contextlib
import math
import time
from collections.abc import Callable, Iterator, Sequence

from .config import RobotConfig
from .errors import I2CError, SafetyError, SensorError, SpeedLimitError
from .hardware.bus import BusLock, null_bus_guard
from .hardware.motor_driver import MotorDriver
from .hardware.registers import MOTOR_ID_COUNT, MOTOR_SPEED_MAX, REG_MOTOR
from .hardware.sensors import LineReading, LineSensorArray
from .hardware.transport import I2CTransport
from .kinematics import DriveAction, wheel_speeds
from .safety import FaultCode, FaultRecord, SafetyController, SafetyState
from .watchdog import MotionWatchdog


class Robot:
    """Physical robot facade over an injected I2C transport."""

    def __init__(
        self,
        transport: I2CTransport,
        config: RobotConfig | None = None,
        *,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        watchdog: bool = True,
        stop_check: Callable[[], bool] | None = None,
        bus_guard: Callable[[], contextlib.AbstractContextManager] | None = None,
        telemetry=None,
    ) -> None:
        self._config = (config or RobotConfig()).validate()
        self._clock = clock
        self._sleep = sleep
        self._transport = transport
        self._stop_check = stop_check
        self._bus_guard_factory = bus_guard
        self._telemetry = telemetry
        self.lock = BusLock()
        self.motors = MotorDriver(
            transport,
            address=self._config.motor.address,
            register=REG_MOTOR,
            lock=self.lock,
            invert=self._config.motor.invert,
        )
        self.sensors = LineSensorArray(
            transport, self._config.sensor, clock=clock, lock=self.lock
        )
        self._safety = SafetyController(
            self._stop_motors_for_safety,
            clock=clock,
            motion_timeout_s=self._config.safety.motion_timeout_s,
        )
        self._watchdog: MotionWatchdog | None = None
        if watchdog:
            self._watchdog = MotionWatchdog(
                lambda: self._on_motion_timeout("watchdog"),
                clock=clock,
                poll_interval_s=self._config.safety.watchdog_period_s,
            )

    # -- construction helpers ----------------------------------------------

    @classmethod
    def connected(
        cls,
        config: RobotConfig | None = None,
        *,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
        watchdog: bool = True,
        stop_check: Callable[[], bool] | None = None,
        bus_guard: Callable[[], contextlib.AbstractContextManager] | None = None,
        telemetry=None,
    ) -> "Robot":
        """Build a robot on the real I2C bus (no motor command is sent)."""
        from .hardware.transport import Smbus2Transport

        resolved = config or RobotConfig()
        transport = Smbus2Transport(resolved.motor.bus)
        return cls(
            transport,
            resolved,
            clock=clock,
            sleep=sleep,
            watchdog=watchdog,
            stop_check=stop_check,
            bus_guard=bus_guard,
            telemetry=telemetry,
        )

    # -- inspection ---------------------------------------------------------

    @property
    def config(self) -> RobotConfig:
        return self._config

    @property
    def clock(self) -> Callable[[], float]:
        """Monotonic clock shared with sensors, safety, and the watchdog."""
        return self._clock

    @property
    def transport(self) -> I2CTransport:
        return self._transport

    @property
    def safety(self) -> SafetyController:
        return self._safety

    @property
    def watchdog(self) -> MotionWatchdog | None:
        return self._watchdog

    @property
    def armed(self) -> bool:
        return self._safety.armed

    @property
    def state(self) -> SafetyState:
        return self._safety.state

    @property
    def fault(self) -> FaultRecord | None:
        return self._safety.fault

    @property
    def emergency_stop_latched(self) -> bool:
        return self._safety.emergency_stop_latched

    @property
    def last_command(self) -> tuple[int, ...] | None:
        return self.motors.last_command

    # -- arming and faults --------------------------------------------------

    def arm(self) -> None:
        """Explicitly permit motion and start the independent watchdog."""
        self._safety.arm()
        if self._watchdog is not None:
            self._watchdog.start()

    def disarm(self) -> None:
        """Refuse further motion and stop the wheels.

        A failed stop latches a communication fault and re-raises; the robot
        stays disarmed either way.
        """
        self._safety.disarm()
        self._cancel_motion_deadline()
        self._stop_motors(context="disarm")

    def set_state(self, state: SafetyState) -> None:
        self._safety.set_state(state)

    def raise_fault(self, code: FaultCode, message: str = "") -> FaultRecord:
        """Latch a fault and stop the wheels (best effort)."""
        return self._latch_and_stop(code, message)

    def clear_fault(self) -> None:
        self._safety.clear_fault()

    def emergency_stop(
        self,
        code: FaultCode = FaultCode.MOTOR_COMMUNICATION_FAILURE,
        message: str = "",
    ) -> FaultRecord:
        """Latch an emergency stop and stop all wheels.

        The latch is set before the stop is attempted, so a failing stop still
        leaves the robot refusing motion; the stop error then propagates.
        """
        return self._safety.emergency_stop(code, message)

    def clear_emergency_stop(self, *, confirm: bool = False) -> None:
        """Release a latched emergency stop; the robot stays disarmed."""
        self._safety.clear_emergency_stop(confirm=confirm)

    # -- sensing ------------------------------------------------------------

    def read_line_sensors(self) -> LineReading:
        """Read and normalise the line sensors (allowed while disarmed)."""
        try:
            with self.lock:
                reading = self.sensors.read()
        except (I2CError, SensorError) as exc:
            self._latch_and_stop(
                FaultCode.SENSOR_FAILURE, f"line sensor read failed: {exc}"
            )
            raise
        if self._telemetry is not None:
            self._telemetry.sensor_reading(reading)
        return reading

    # -- motion -------------------------------------------------------------

    def drive(self, action: DriveAction | str, speed: int) -> tuple[int, ...]:
        """Run one drive primitive. Returns the per-wheel signed speeds."""
        resolved = DriveAction(action)
        self._check_speed(speed)
        return self.drive_wheels(
            wheel_speeds(resolved, speed), context=resolved.value
        )

    def forward(self, speed: int) -> tuple[int, ...]:
        return self.drive(DriveAction.FORWARD, speed)

    def backward(self, speed: int) -> tuple[int, ...]:
        return self.drive(DriveAction.BACKWARD, speed)

    def strafe_left(self, speed: int) -> tuple[int, ...]:
        return self.drive(DriveAction.STRAFE_LEFT, speed)

    def strafe_right(self, speed: int) -> tuple[int, ...]:
        return self.drive(DriveAction.STRAFE_RIGHT, speed)

    def rotate_left(self, speed: int) -> tuple[int, ...]:
        return self.drive(DriveAction.ROTATE_LEFT, speed)

    def rotate_right(self, speed: int) -> tuple[int, ...]:
        return self.drive(DriveAction.ROTATE_RIGHT, speed)

    def drive_wheels(
        self, speeds: Sequence[int], *, context: str = "drive"
    ) -> tuple[int, ...]:
        """Drive the four wheels with explicit signed counts (motor-id order).

        Permission checks, the deadline refresh, and the write happen inside one
        transaction, so a stop that latches concurrently cannot be followed by a
        nonzero write.
        """
        self._check_wheel_speeds(speeds)
        values = tuple(speeds)
        try:
            with self._transaction(context):
                self._check_external_stop()
                self._safety.require_armed()
                self._note_motion()
                self.motors.set_speeds(values)
        except I2CError as exc:
            self._latch_and_stop(
                FaultCode.MOTOR_COMMUNICATION_FAILURE,
                f"{context} command failed: {exc}",
            )
            raise
        if self._telemetry is not None:
            self._telemetry.motor_command(values, context=context)
        return values

    def run_command(
        self,
        command: Callable[[], tuple[int, ...]],
        duration_s: float,
        *,
        refresh_period_s: float | None = None,
        deadline: float | None = None,
        stop_event=None,
        context: str = "hold",
    ) -> float:
        """Hold a bounded command, refreshing it faster than the watchdog.

        Used by timed manoeuvres (wheel probes, TURN, DOCK, recovery phases) so a
        long hold never lets the motion deadline expire. Always stops in
        ``finally`` and returns the elapsed seconds.
        """
        self._validate_duration(duration_s, "duration_s")
        if refresh_period_s is not None:
            self._validate_duration(refresh_period_s, "refresh_period_s")
        period = refresh_period_s or max(
            0.02, min(0.1, self._safety.motion_timeout_s / 3.0)
        )
        if period >= self._safety.motion_timeout_s:
            raise ValueError(
                "refresh period must be shorter than the motion watchdog "
                f"timeout ({period} >= {self._safety.motion_timeout_s})"
            )
        started = self._clock()
        # A wall-clock bound keeps the hold bounded even if the injected clock is
        # frozen (tests) or jumps backwards.
        real_started = time.monotonic()
        end = started + float(duration_s)
        if deadline is not None:
            end = min(end, float(deadline))
        primary_error: BaseException | None = None
        try:
            while True:
                now = self._clock()
                real_elapsed = time.monotonic() - real_started
                if now >= end or real_elapsed >= float(duration_s):
                    break
                if stop_event is not None and stop_event.is_set():
                    break
                command()
                remaining = end - self._clock()
                if remaining <= 0:
                    break
                self._sleep(min(period, remaining))
        except BaseException as exc:
            # Remember whether something else already failed so the final stop
            # cannot hide it, and so a stop failure is reported when nothing did.
            primary_error = exc
            raise
        finally:
            try:
                self.stop()
            except I2CError as stop_error:
                if primary_error is None:
                    # `stop` already latched the failure inside the robot; the
                    # caller must still learn that the hold did not end cleanly.
                    raise
                self._safety.note_failure(
                    f"final stop failed during run_command: {stop_error}"
                )
        return self._clock() - started

    def stop(self) -> None:
        """Stop all wheels. Always permitted, armed or not."""
        self._cancel_motion_deadline()
        self._stop_motors(context="stop")

    def tick(self) -> bool:
        """Synchronously report deadline expiry; never cancels a live deadline.

        The independent watchdog covers a controller that never calls this. When
        the deadline *has* expired, this latches ``MOTION_TIMEOUT`` and stops.
        """
        if not self._safety.deadline_expired():
            return False
        self._on_motion_timeout("tick")
        return True

    # -- lifecycle ----------------------------------------------------------

    def close(self) -> None:
        """Stop the watchdog thread, stop the wheels, and close the transport."""
        if self._watchdog is not None:
            self._watchdog.stop()
        self._safety.disarm()
        self._cancel_motion_deadline()
        try:
            self._stop_motors(context="close")
        finally:
            self._close_transport()

    def __enter__(self) -> "Robot":
        return self

    def __exit__(self, *_exc_info) -> None:
        self.close()

    # -- internals ----------------------------------------------------------

    @contextlib.contextmanager
    def _transaction(self, context: str) -> Iterator[None]:
        """Serialize permission checks and the bus write."""
        factory = self._bus_guard_factory or null_bus_guard
        with self.lock:
            try:
                with factory():
                    yield
            except TimeoutError as exc:
                self._safety.raise_fault(
                    FaultCode.MOTOR_COMMUNICATION_FAILURE,
                    f"{context}: timed out waiting for the cross-process bus lock",
                )
                raise I2CError(
                    f"{context}: timed out waiting for the cross-process bus lock"
                ) from exc

    def _close_transport(self) -> None:
        closer = getattr(self._transport, "close", None)
        if closer is None:
            return
        try:
            closer()
        except I2CError as exc:
            self._safety.note_failure(f"transport close failed: {exc}")
            raise

    def _validate_duration(self, value, name: str) -> float:
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value < 0
        ):
            raise ValueError(f"{name} must be a finite, non-negative number")
        return float(value)

    def _check_speed(self, speed: int) -> None:
        if isinstance(speed, bool) or not isinstance(speed, int):
            raise SpeedLimitError(
                f"speed must be an int PWM magnitude, got {speed!r}"
            )
        limit = self._config.motor.max_speed
        if not 0 <= speed <= limit:
            raise SpeedLimitError(
                f"speed {speed} outside configured range 0..{limit}"
            )
        if speed > MOTOR_SPEED_MAX:
            raise SpeedLimitError(
                f"speed {speed} exceeds the protocol maximum {MOTOR_SPEED_MAX}"
            )

    def _check_wheel_speeds(self, speeds: Sequence[int]) -> None:
        if len(speeds) != MOTOR_ID_COUNT:
            raise SpeedLimitError(
                f"expected {MOTOR_ID_COUNT} wheel speeds, got {len(speeds)}"
            )
        limit = self._config.motor.max_speed
        for index, value in enumerate(speeds):
            if isinstance(value, bool) or not isinstance(value, int):
                raise SpeedLimitError(
                    f"wheel {index} speed must be an int count, got {value!r}"
                )
            if abs(value) > limit:
                raise SpeedLimitError(
                    f"wheel {index} speed {value} exceeds configured magnitude "
                    f"{limit}"
                )

    def _note_motion(self) -> None:
        with self.lock:
            self._safety.note_motion()
            if self._watchdog is not None:
                self._watchdog.refresh(self._safety.motion_timeout_s)

    def _cancel_motion_deadline(self) -> None:
        with self.lock:
            self._safety.clear_motion()
            if self._watchdog is not None:
                self._watchdog.cancel()

    def _external_stop_requested(self) -> bool:
        if self._stop_check is None:
            return False
        try:
            return bool(self._stop_check())
        except Exception:
            # A broken stop channel must never be read as "no stop requested".
            return True

    def _check_external_stop(self) -> None:
        """Refuse motion when another process asked for a stop."""
        if not self._external_stop_requested():
            return
        self._safety.disarm()
        self._cancel_motion_deadline()
        try:
            with self._transaction("stop request"):
                self.motors.stop_all()
        except I2CError as exc:
            self._safety.note_failure(f"stop request stop failed: {exc}")
        raise SafetyError("motion refused: another process requested a stop")

    def _stop_motors(self, *, context: str) -> None:
        """Stop every wheel inside the transaction, latching on failure."""
        try:
            with self._transaction(context):
                self.motors.stop_all()
        except I2CError as exc:
            self._safety.note_failure(f"{context} failed to stop the wheels: {exc}")
            if self._telemetry is not None:
                self._telemetry.stop(f"{context}:failed")
            raise
        if self._telemetry is not None:
            self._telemetry.stop(context)

    def _stop_motors_for_safety(self) -> None:
        """Stop callback used by :class:`SafetyController`."""
        try:
            with self._transaction("safety stop"):
                self.motors.stop_all()
        except I2CError as exc:
            self._safety.note_failure(f"stop attempt failed: {exc}")
            raise

    def _latch_and_stop(self, code: FaultCode, message: str) -> FaultRecord:
        """Latch ``code`` first, then attempt a stop without hiding the cause."""
        record = self._safety.raise_fault(code, message)
        self._cancel_motion_deadline()
        self._log_fault(record)
        try:
            self._stop_motors(context="fault stop")
        except I2CError:
            # `_stop_motors` already appended the failure to the existing fault.
            pass
        return record

    def _on_motion_timeout(self, source: str) -> None:
        """Shared timeout handler for the watchdog thread and :meth:`tick`."""
        # Re-check under the command lock: a refresh that landed after the
        # watchdog claimed expiry must not become a stale timeout.
        with self.lock:
            if not self._safety.deadline_expired():
                return
            if self._watchdog is not None:
                self._watchdog.cancel()
            record = self._safety.raise_fault(
                FaultCode.MOTION_TIMEOUT,
                f"motion deadline expired ({source}) without a refresh from the "
                "controller",
            )
            self._log_fault(record)
            try:
                self._stop_motors(context="watchdog stop")
            except I2CError:
                pass

    def _log_fault(self, record: FaultRecord) -> None:
        if self._telemetry is None:
            return
        try:
            self._telemetry.fault(record)
        except Exception:
            # Telemetry must never break a safety path.
            pass


__all__ = ["Robot"]
