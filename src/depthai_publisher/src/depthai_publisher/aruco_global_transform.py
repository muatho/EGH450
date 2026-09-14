#!/usr/bin/env python3

import cv2
import numpy as np

from depthai_publisher.camera_to_map import CameraToMap


class ArucoGlobalTransformer(CameraToMap):
    """Compose camera PnP and vehicle pose into a map-frame marker pose.

    Transform notation follows T_A_B: it maps coordinates expressed in frame B
    into frame A. OpenCV solvePnP supplies T_camera_marker. MAVROS supplies
    T_map_drone. The fixed measured camera mount supplies T_drone_camera.

    Transformation chain:

        T_map_marker =
            T_map_drone
            @ T_drone_camera
            @ T_camera_marker
    """

    def build_camera_marker_transform(
        self,
        rvec,
        tvec
    ):
        """Build T_camera_marker from OpenCV solvePnP output.

        solvePnP returns the transform that maps the marker's object points into
        the optical camera frame. Rodrigues converts its compact rotation vector
        into the 3 x 3 matrix required by a homogeneous transform.
        """

        # Rodrigues vector -> rotation matrix.
        R_camera_marker, _ = cv2.Rodrigues(rvec)

        T = np.eye(4)

        T[:3, :3] = R_camera_marker
        T[:3, 3] = tvec.flatten()

        return T

    def camera_to_world(self, rvec, tvec, pose_msg):
        """Return T_map_marker using one synchronized vehicle pose.

        Returns
        -------
        numpy.ndarray or None
            A 4 x 4 homogeneous transform from marker coordinates into map
            coordinates. None is returned only when no pose message is supplied.
        """

        # Construct each adjacent link in the frame chain independently so its
        # direction and source remain visible during debugging.
        T_map_drone = self.build_map_drone_transform(pose_msg)
        if T_map_drone is None:
            return None

        T_drone_camera = self.build_drone_camera_transform()

        T_camera_marker = self.build_camera_marker_transform(
            rvec,
            tvec
        )

        # Matrix multiplication applies right-to-left: marker -> camera ->
        # drone -> map.
        T_map_marker = (
            T_map_drone
            @ T_drone_camera
            @ T_camera_marker
        )

        return T_map_marker


    def print_marker_pose(self, marker_position):
        """Print a map-frame position in metres for terminal debugging."""

        print(
            f"World X: {marker_position[0]:.2f} m | "
            f"Y: {marker_position[1]:.2f} m | "
            f"Z: {marker_position[2]:.2f} m"
        )
