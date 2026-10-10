"""
Shared value types used by the autonomous mission.

These immutable types provide explicit names for mission data that was
previously represented by positional lists and loosely related variables.
"""

import math

from dataclasses import dataclass


@dataclass(frozen=True)
class Waypoint:
    """A validated map-frame flight destination."""

    x: float
    y: float
    z: float
    yaw: float = 0.0

    def __post_init__(self):
        values = (
            self.x,
            self.y,
            self.z,
            self.yaw,
        )

        if not all(math.isfinite(value) for value in values):
            raise ValueError(
                "Waypoint coordinates and yaw must be finite"
            )