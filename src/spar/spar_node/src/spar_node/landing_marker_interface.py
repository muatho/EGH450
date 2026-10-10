"""Persistent ArUco landing-marker observations and terminal selection."""

from dataclasses import dataclass
from typing import Dict, Optional

import math
import threading

import rospy
from visualization_msgs.msg import MarkerArray


@dataclass(frozen=True)
class LandingMarker:
    """One persistent map-frame landing-marker location."""

    marker_id: int
    x: float
    y: float
    z: float


class LandingMarkerInterface:
    """Store observed ArUcos and collect the legacy terminal selection."""

    def __init__(self):
        self._lock = threading.RLock()
        self._markers: Dict[int, LandingMarker] = {}
        self._selection_started = False
        self._selected_marker_id = None
        self._topic = rospy.get_param("~landing_marker_topic", "/world_model/aruco_markers")
        self._subscriber = rospy.Subscriber(self._topic, MarkerArray, self._marker_callback, queue_size=1)

    def _marker_callback(self, message: MarkerArray) -> None:
        """Refresh the persistent marker locations published by vision."""

        with self._lock:
            for marker in message.markers:
                coordinates = (marker.pose.position.x, marker.pose.position.y, marker.pose.position.z)
                if not all(math.isfinite(value) for value in coordinates):
                    rospy.logwarn("Ignoring ArUco %i with a non-finite map position", marker.id)
                    continue

                self._markers[int(marker.id)] = LandingMarker(
                    marker_id=int(marker.id), x=coordinates[0], y=coordinates[1], z=coordinates[2]
                )

        rospy.loginfo_throttle(5.0, "[ARUCO] %i persistent landing marker(s) available", len(self._markers))

    def begin_selection(self) -> None:
        """Start one non-blocking terminal prompt for a landing-marker ID."""

        with self._lock:
            if self._selection_started:
                return
            self._selection_started = True
            marker_ids = sorted(self._markers)

        rospy.loginfo("[LAND] Search complete. Hovering while awaiting landing ArUco selection.")
        if marker_ids:
            rospy.loginfo("[LAND] Available ArUco IDs: %s", ", ".join(str(marker_id) for marker_id in marker_ids))
        else:
            rospy.logwarn("[LAND] No ArUco landing markers have been stored yet")

        thread = threading.Thread(target=self._wait_for_selection, name="landing_marker_selection", daemon=True)
        thread.start()

    def _wait_for_selection(self) -> None:
        """Prompt until the operator enters an observed numeric marker ID."""

        while not rospy.is_shutdown():
            try:
                marker_id = int(input("Enter landing ArUco ID: "))
            except ValueError:
                print("Please enter a numeric ArUco ID.")
                continue
            except EOFError:
                rospy.logerr("[LAND] Mission pane input closed while awaiting an ArUco ID")
                return

            with self._lock:
                if marker_id not in self._markers:
                    available = sorted(self._markers)
                else:
                    self._selected_marker_id = marker_id
                    rospy.loginfo("[LAND] ArUco %i selected for landing", marker_id)
                    return

            print("ArUco {} has not been detected. Available IDs: {}".format(marker_id, available))

    def selected_marker(self) -> Optional[LandingMarker]:
        """Return the selected marker, or None while selection is pending."""

        with self._lock:
            if self._selected_marker_id is None:
                return None
            return self._markers.get(self._selected_marker_id)
