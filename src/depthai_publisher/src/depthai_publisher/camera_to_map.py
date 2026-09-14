#!/usr/bin/env python3

"""Shared conversion from OAK optical-camera coordinates to ROS map."""

import numpy as np
from scipy.spatial.transform import Rotation as R


class CameraToMap:
    """Apply the measured camera mount and one vehicle pose.

    Transform notation follows T_A_B: the matrix maps coordinates expressed in
    frame B into frame A. Camera points use optical axes +X right, +Y down and
    +Z forward; drone/body coordinates use ROS FLU axes.
    """

    # Camera origin expressed in drone/body FLU metres.
    CAMERA_X = 0.00
    CAMERA_Y = 0.00
    CAMERA_Z = -0.08

    # Downward-facing mount confirmed during the ArUco transform diagnosis:
    # body +X forward = optical -Y, body +Y left = optical -X,
    # body +Z up = optical -Z.
    CAMERA_ROTATION = np.array([
        [0, -1, 0],
        [-1, 0, 0],
        [0, 0, -1],
    ], dtype=float)

    def build_map_drone_transform(self, pose_msg):
        """Build T_map_drone from a ROS PoseStamped message."""
        if pose_msg is None:
            return None

        position = np.array([
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z,
        ], dtype=float)
        quaternion = np.array([
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        ], dtype=float)

        transform = np.eye(4)
        transform[:3, :3] = R.from_quat(quaternion).as_matrix()
        transform[:3, 3] = position
        return transform

    def build_drone_camera_transform(self):
        """Build fixed T_drone_camera from the physical camera mounting."""
        transform = np.eye(4)
        transform[:3, :3] = self.CAMERA_ROTATION
        transform[:3, 3] = np.array([
            self.CAMERA_X,
            self.CAMERA_Y,
            self.CAMERA_Z,
        ])
        return transform

    def camera_point_to_map(self, camera_point, pose_msg):
        """Return one optical-camera XYZ point expressed in map coordinates."""
        map_drone = self.build_map_drone_transform(pose_msg)
        if map_drone is None:
            return None

        camera_point_h = np.ones(4)
        camera_point_h[:3] = np.asarray(camera_point, dtype=float).reshape(3)
        map_point_h = (
            map_drone
            @ self.build_drone_camera_transform()
            @ camera_point_h
        )
        return map_point_h[:3]
