"""
High-level search geometry for the autonomous mission.

This module describes where the aircraft must search. It does not calculate
obstacle avoidance or send flight commands. Breadcrumb remains responsible
for finding a safe route between consecutive search destinations.

Legacy successor:
    The fixed `waypoints` lawnmower list created in
    test_breadcrumb_camera.main().
"""

import math

from typing import List

from spar_node.mission_types import Waypoint


def build_default_search_plan(
    survey_altitude: float,
) -> List[Waypoint]:
    """
    Return the established lawnmower search pattern.

    The first point anchors the search at the map origin. If take-off already
    leaves the aircraft at that point, the mission controller may recognise it
    as reached rather than requesting unnecessary movement.

    Args:
        survey_altitude: Height used for every search destination.

    Returns:
        Ordered high-level search destinations.
    """

    if (
        not math.isfinite(survey_altitude)
        or survey_altitude <= 0
    ):
        raise ValueError(
            "survey_altitude must be finite and greater than zero"
        )

    # Preserve the established search geometry. These are high-level mission
    # destinations; Breadcrumb expands each leg into an obstacle-aware route.
    search_coordinates = (
        (0.0, 0.0),
        (-3.3, -2.5),
        (3.3, -2.5),
        (3.3, -1.5),
        (-3.3, -1.5),
        (-3.3, -0.5),
        (3.3, -0.5),
        (3.3, 0.5),
        (-3.3, 0.5),
        (-3.3, 1.5),
        (3.3, 1.5),
    )

    return [
        Waypoint(
            x=x,
            y=y,
            z=survey_altitude,
            yaw=0.0,
        )
        for x, y in search_coordinates
    ]