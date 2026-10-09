"""Small PID line-following package; importing it never moves the robot."""

from .config import RobotConfig
from .robot import Robot

__all__ = ["Robot", "RobotConfig"]
__version__ = "0.3.6"
