"""
ROS service boundary for the Breadcrumb path planner.

Breadcrumb remains responsible for calculating obstacle-avoiding paths.
This interface owns service discovery and communication so the mission
controller only needs to express when a path is required.
"""

import rospy
import math

from typing import List, Optional

from geometry_msgs.msg import Point
from spar_node.mission_types import Waypoint
from breadcrumb.srv import RequestPath, RequestPathRequest

from breadcrumb.srv import RequestPath


class BreadcrumbInterface:
    """Provide controlled access to the Breadcrumb path-planning service."""

    def __init__(self):
        # Retain the established service used by test_breadcrumb_camera while
        # allowing deployment-specific remapping through a ROS parameter.
        self._service_name = rospy.get_param(
            "~breadcrumb_service",
            "/breadcrumb/request_path",
        )

        self._client = rospy.ServiceProxy(
            self._service_name,
            RequestPath,
        )

        self._connected = False

        rospy.loginfo(
            "Breadcrumb interface created for service: %s",
            self._service_name,
        )

    @property
    def is_connected(self) -> bool:
        """Return whether Breadcrumb passed the latest readiness check."""
        return self._connected

    def wait_until_ready(self, timeout_seconds: float) -> bool:
        """
        Wait for Breadcrumb to become available within a finite timeout.

        Args:
            timeout_seconds: Maximum number of seconds to wait. It must be
                greater than zero.

        Returns:
            True when the Breadcrumb service is available.
            False when the timeout expires first.
        """

        if timeout_seconds <= 0:
            raise ValueError("timeout_seconds must be greater than zero")

        rospy.loginfo(
            "Waiting up to %.1f seconds for Breadcrumb on %s",
            timeout_seconds,
            self._service_name,
        )

        try:
            rospy.wait_for_service(
                self._service_name,
                timeout=timeout_seconds,
            )
        except rospy.ROSException:
            self._connected = False
            rospy.logerr(
                "Breadcrumb service was unavailable after %.1f seconds",
                timeout_seconds,
            )
            return False

        self._connected = True
        rospy.loginfo("Breadcrumb path-planning service is available")
        return True

    def request_path(
        self,
        start: Point,
        destination: Point,
    ) -> Optional[List[Waypoint]]:
        """
        Request an obstacle-avoiding path between two map-frame positions.

        Breadcrumb calculates both a full grid path and a sparse path with
        unnecessary intermediate points removed. This method preserves the
        established mission behaviour of preferring the sparse path and
        falling back to the full path when required.

        Args:
            start: Current or previous map-frame position.
            destination: Required map-frame destination.

        Returns:
            The selected list of Breadcrumb poses.
            None when the service fails or no route exists.
        """

        coordinates = (
            start.x,
            start.y,
            start.z,
            destination.x,
            destination.y,
            destination.z,
        )

        if not all(math.isfinite(value) for value in coordinates):
            rospy.logerr(
                "Refusing Breadcrumb request with invalid coordinates"
            )
            return None

        request = RequestPathRequest()

        request.start.x = start.x
        request.start.y = start.y
        request.start.z = start.z

        request.end.x = destination.x
        request.end.y = destination.y
        request.end.z = destination.z

        rospy.loginfo(
            "Requesting Breadcrumb path: "
            "[%.2f, %.2f, %.2f] -> [%.2f, %.2f, %.2f]",
            request.start.x,
            request.start.y,
            request.start.z,
            request.end.x,
            request.end.y,
            request.end.z,
        )

        try:
            response = self._client(request)
        except rospy.ServiceException as error:
            rospy.logerr(
                "Breadcrumb service call failed: %s",
                str(error),
            )
            return None

        # Preserve the established Breadcrumb selection policy: use the sparse
        # route whenever available, fall back to the full grid route, and
        # report failure when neither contains a valid solution.
        normal_path = response.path.poses
        sparse_path = response.path_sparse.poses

        rospy.loginfo(
            "Breadcrumb returned %i normal poses and %i sparse poses",
            len(normal_path),
            len(sparse_path),
        )

        if sparse_path:
            rospy.loginfo("Using sparse Breadcrumb path")
            selected_path = sparse_path
        elif normal_path:
            rospy.logwarn(
                "Sparse Breadcrumb path is empty; using the normal path"
            )
            selected_path = normal_path
        else:
            rospy.logerr(
                "Breadcrumb could not find a path between the "
                "requested positions"
            )
            return None

        try:
            # Preserve the legacy conversion from Breadcrumb poses to flight
            # waypoints. Breadcrumb controls x, y and z; route yaw remains zero.
            return [
                Waypoint(
                    x=pose.position.x,
                    y=pose.position.y,
                    z=pose.position.z,
                    yaw=0.0,
                )
                for pose in selected_path
            ]
        except ValueError as error:
            rospy.logerr(
                "Breadcrumb returned an invalid waypoint: %s",
                str(error),
            )
            return None