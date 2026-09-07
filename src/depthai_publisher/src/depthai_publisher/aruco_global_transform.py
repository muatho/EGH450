#!/usr/bin/env python3

import cv2
import numpy as np

from scipy.spatial.transform import Rotation as R


class ArucoGlobalTransformer:
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

    # ============================================================
    # Camera mounting parameters
    # ============================================================

    # Camera origin expressed in the drone body frame, in metres. Body axes are
    # assumed to follow ROS FLU: +X forward, +Y left, +Z upward.
    CAMERA_X = 0.00      # +forward
    CAMERA_Y = 0.00      # +left
    CAMERA_Z = -0.08     # below body origin

    # OpenCV optical axes are +X image-right, +Y image-down, +Z through the
    # lens. For the measured downward-facing mounting on this drone:
    # body X (forward) = -camera Y (image up)
    # body Y (left)    = -camera X (image left)
    # body Z (up)      = -camera Z (opposite the downward optical axis)

    CAMERA_ROTATION = np.array([

        [0, -1, 0],
        [-1, 0, 0],
        [0, 0, -1]

    ], dtype=float)

    # ============================================================

    def build_map_drone_transform(self, pose_msg):
        """Build T_map_drone from the synchronized MAVROS PoseStamped.

        The synchronizer in aruco_subscriber supplies this message explicitly;
        this class no longer owns an asynchronous "latest pose" subscriber.
        MAVROS position is the body origin expressed in map; its quaternion
        rotates body-frame vectors into map-frame vectors.
        """
        if pose_msg is None:
            return None

        position = np.array([
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z,
        ])
        orientation = np.array([
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        ])

        # scipy expects quaternion components in ROS order: x, y, z, w.
        R_map_drone = R.from_quat(
            orientation
        ).as_matrix()

        # A homogeneous transform stores rotation in its upper-left 3 x 3 and
        # translation in its upper-right 3 x 1 column.
        T = np.eye(4)

        T[:3, :3] = R_map_drone
        T[:3, 3] = position

        return T


    def build_drone_camera_transform(self):
        """Build fixed T_drone_camera from the measured camera mounting.

        Despite the readable name "drone_camera", this matrix maps a point
        from camera coordinates into drone coordinates. Its translation is the
        camera origin expressed in the drone body frame.
        """

        R_drone_camera = self.CAMERA_ROTATION

        T = np.eye(4)

        T[:3, :3] = R_drone_camera

        T[:3, 3] = np.array([
            self.CAMERA_X,
            self.CAMERA_Y,
            self.CAMERA_Z
        ])

        return T


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
