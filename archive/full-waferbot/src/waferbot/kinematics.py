"""Wheel-level mixing for the Yahboom Raspbot V2 mecanum chassis.

The signatures below are derived from vendored source, not from guesswork.

Evidence chain
--------------
``vendor/yahboom/project_demo/lib/McLumk_Wheel_Sports.py``::

    def set_deflection(speed, deflection):        # 0=right 90=fwd 180=left 270=back
        vx = speed * cos(deflection)
        vy = speed * sin(deflection)
        l1 = int(vy + vx)
        l2 = int(vy - vx)
        r1 = int(vy - vx)
        r2 = int(vy + vx)
        return l1, l2, r1, r2

with ``move_forward = set_deflection(speed, 90)``,
``move_backward = ...270``, ``move_left = ...180``, ``move_right = ...0``.
``rotate_left``/``rotate_right`` reuse the 180/0 deflection sign patterns but
then force both left wheels to share a sign and both right wheels to the
opposite sign (``-l2`` / ``abs(r2)`` and ``abs(l2)`` / ``-r2``), which is why
rotation is a differential spin rather than a strafe.

``vendor/yahboom/project_demo/raspbot/Raspbot_Lib.py`` fixes the motor ids:
``0 = L1``, ``1 = L2``, ``2 = R1``, ``3 = R2``.

``vendor/yahboom/project_demo/09.AI_Big_Model/AI_CarAgent_en/Car_base_control.py``
confirms the intent of each action ("Move left translation", "Turn left in
place"), so strafing and rotation are deliberately different wheel patterns.

Evaluating the vendor formula with integer ``speed`` gives exact integer
results at the four cardinal deflections, which is what the tuples below
encode. Signs are relative to the vendored ``Ctrl_Muto`` convention
(positive = that wheel drives forward).
"""

from __future__ import annotations

from enum import Enum

from .hardware.registers import MOTOR_ID_COUNT, MOTOR_SPEED_MAX


class DriveAction(str, Enum):
    """High-level drive primitives exposed by :class:`waferbot.robot.Robot`."""

    FORWARD = "forward"
    BACKWARD = "backward"
    STRAFE_LEFT = "strafe_left"
    STRAFE_RIGHT = "strafe_right"
    ROTATE_LEFT = "rotate_left"
    ROTATE_RIGHT = "rotate_right"


#: Per-wheel sign for each action, in motor-id order (0=L1, 1=L2, 2=R1, 3=R2).
WHEEL_SIGNATURES: dict[DriveAction, tuple[int, int, int, int]] = {
    # set_deflection(speed, 90)  -> (+s, +s, +s, +s)
    DriveAction.FORWARD: (1, 1, 1, 1),
    # set_deflection(speed, 270) -> (-s, -s, -s, -s)
    DriveAction.BACKWARD: (-1, -1, -1, -1),
    # set_deflection(speed, 180) -> (-s, +s, +s, -s)
    DriveAction.STRAFE_LEFT: (-1, 1, 1, -1),
    # set_deflection(speed, 0)   -> (+s, -s, -s, +s)
    DriveAction.STRAFE_RIGHT: (1, -1, -1, 1),
    # rotate_left: set_deflection(speed, 180) with l2 negated, r2 absolute
    DriveAction.ROTATE_LEFT: (-1, -1, 1, 1),
    # rotate_right: set_deflection(speed, 0) with l2 absolute, r2 negated
    DriveAction.ROTATE_RIGHT: (1, 1, -1, -1),
}


def wheel_speeds(
    action: DriveAction, speed: int
) -> tuple[int, int, int, int]:
    """Return signed wheel speeds in motor-id order for ``action``.

    ``speed`` is a PWM magnitude in counts. The value is validated rather than
    clamped: a caller that asks for more than the protocol supports has a bug
    that clamping would hide.
    """
    if isinstance(speed, bool) or not isinstance(speed, int):
        raise ValueError(f"speed must be an int, got {type(speed).__name__}")
    if not 0 <= speed <= MOTOR_SPEED_MAX:
        raise ValueError(
            f"speed {speed} outside supported range 0..{MOTOR_SPEED_MAX}"
        )
    try:
        signature = WHEEL_SIGNATURES[action]
    except KeyError as exc:
        raise ValueError(f"unsupported drive action: {action!r}") from exc
    if len(signature) != MOTOR_ID_COUNT:
        raise AssertionError(
            f"wheel signature for {action} has {len(signature)} entries"
        )
    return tuple(sign * speed for sign in signature)  # type: ignore[return-value]


__all__ = ["DriveAction", "WHEEL_SIGNATURES", "wheel_speeds"]

