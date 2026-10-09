"""The two tape boundaries the PID follower can track."""

from enum import Enum


class EdgeState(str, Enum):
    BLACK_LEFT = "BLACK_LEFT"
    BLACK_RIGHT = "BLACK_RIGHT"
