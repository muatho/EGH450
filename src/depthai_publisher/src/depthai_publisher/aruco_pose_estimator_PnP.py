#!/usr/bin/env python3

import cv2
import numpy as np


class ArucoPoseEstimator():
    """Estimate a square ArUco's optical-camera pose from its four corners.

    This class owns camera calibration and marker geometry, but it deliberately
    does not know about MAVROS or global frames. Its output is the OpenCV
    transform T_camera_marker, represented by Rodrigues ``rvec`` and translation
    ``tvec``. ArucoGlobalTransformer handles the later map-frame composition.

    A square is planar and can produce two plausible PnP solutions. IPPE Square
    returns those candidates explicitly; this class rejects candidates behind
    the camera and selects the one with the smallest corner reprojection error.
    """

    def __init__(self, marker_size=0.200, max_reprojection_error=3.0):
        """Define marker geometry and the maximum accepted pixel error.

        ``marker_size`` is the physical side length of the black marker square
        in metres. Translation returned by PnP scales directly with this value.
        ``max_reprojection_error`` is the maximum RMS corner error, in pixels,
        accepted after projecting an estimated pose back into the image.
        """
        self.camera_matrix = None
        self.dist_coeffs = None
        self.got_camera_info = False

        self.marker_size = marker_size
        self.max_reprojection_error = max_reprojection_error

        # Updated after every successful estimate and cleared on rejection.
        # This makes solver quality inspectable without changing the established
        # (success, rvec, tvec) return interface used by aruco_subscriber.py.
        self.last_reprojection_error = None

        s = self.marker_size

        # SOLVEPNP_IPPE_SQUARE requires this exact object-point order and marker
        # coordinate convention. cv2.aruco supplies detected image corners in
        # the matching semantic order: top-left, top-right, bottom-right,
        # bottom-left. Marker +X points right, +Y points up, and Z=0 is its plane.
        self.obj_points = np.array([
            [-s/2,  s/2, 0],
            [ s/2,  s/2, 0],
            [ s/2, -s/2, 0],
            [-s/2, -s/2, 0]
        ], dtype=np.float32)

    def set_camera_info(self, msg):
        """Copy K and D from the latest CameraInfo used by this image stream."""
        self.camera_matrix = np.array([
            [msg.K[0], msg.K[1], msg.K[2]],
            [msg.K[3], msg.K[4], msg.K[5]],
            [msg.K[6], msg.K[7], msg.K[8]]
        ], dtype=np.float32)

        # OpenCV accepts the eight rational-polynomial CameraInfo values. They
        # must describe the same pixels used for marker detection.
        self.dist_coeffs = np.array(
            msg.D,
            dtype=np.float32
        )

        self.got_camera_info = True

    def estimate_pose(self, marker_corner):
        """Return the best valid IPPE-square pose for one detected marker.

        The result is rejected when calibration is unavailable, OpenCV returns
        no solution, all candidates lie behind the camera, values are non-finite,
        or the best candidate's RMS reprojection error exceeds the configured
        pixel limit.
        """
        self.last_reprojection_error = None

        if not self.got_camera_info:
            return False, None, None

        # ArUcoDetector returns shape (1, 4, 2); PnP expects four 2D points.
        img_points = marker_corner.reshape((4, 2)).astype(np.float32)

        # IPPE Square is specialized for four coplanar square corners and
        # returns both planar pose candidates instead of hiding the ambiguity.
        solution_count, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            self.obj_points,
            img_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if solution_count < 1:
            return False, None, None

        valid_candidates = []

        for rvec, tvec in zip(rvecs, tvecs):
            rvec = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
            tvec = np.asarray(tvec, dtype=np.float64).reshape(3, 1)

            # A visible marker must be in front of the optical camera, where
            # OpenCV defines positive Z. NaN/Inf values are never publishable.
            if not np.all(np.isfinite(rvec)) or not np.all(np.isfinite(tvec)):
                continue
            if tvec[2, 0] <= 0.0:
                continue

            # Reproject the known 3D corners through this candidate pose. RMS
            # pixel distance from the detected corners measures how well the
            # full camera model and pose explain the observation.
            projected, _ = cv2.projectPoints(
                self.obj_points,
                rvec,
                tvec,
                self.camera_matrix,
                self.dist_coeffs,
            )
            projected = projected.reshape((4, 2))
            residuals = projected - img_points
            rms_error = float(
                np.sqrt(np.mean(np.sum(residuals * residuals, axis=1)))
            )

            valid_candidates.append((rms_error, rvec, tvec))

        if not valid_candidates:
            return False, None, None

        # The lowest-error candidate is the pose most consistent with all four
        # observed corners. Error is the first tuple element used by min().
        best_error, best_rvec, best_tvec = min(
            valid_candidates,
            key=lambda candidate: candidate[0],
        )

        if best_error > self.max_reprojection_error:
            return False, None, None

        self.last_reprojection_error = best_error
        return True, best_rvec, best_tvec

    def draw_pose_overlay(
        self,
        frame,
        rvec,
        tvec,
        top_left
    ):
        """Draw marker axes, camera-frame translation, and fit error."""
        # Axes are drawn using the same K and D used to estimate the pose.
        cv2.drawFrameAxes(
            frame,
            self.camera_matrix,
            self.dist_coeffs,
            rvec,
            tvec,
            0.1
        )

        # Translation is the marker origin expressed in optical-camera metres.
        Xc = tvec[0][0]
        Yc = tvec[1][0]
        Zc = tvec[2][0]

        pose_lines = (
            f"err: {self.last_reprojection_error:.2f}px",
            f"Z: {Zc:.2f}",
            f"Y: {Yc:.2f}",
            f"X: {Xc:.2f}"
        )
        first_y = max(top_left[1] - 70, 15)
        for line_number, pose_text in enumerate(pose_lines):
            self.draw_text_180(
                frame,
                pose_text,
                (top_left[0], first_y + line_number * 18),
                cv2.FONT_HERSHEY_COMPLEX,
                0.5,
                (0, 255, 255),
                2,
            )

    @staticmethod
    def draw_text_180(frame, text, origin, font, scale, color, thickness):
        """Draw one text line rotated 180 degrees inside its local box."""
        (width, height), baseline = cv2.getTextSize(
            text, font, scale, thickness
        )
        padding = 2
        mask = np.zeros(
            (height + baseline + 2 * padding, width + 2 * padding),
            dtype=np.uint8,
        )
        cv2.putText(
            mask,
            text,
            (padding, height + padding),
            font,
            scale,
            255,
            thickness,
            cv2.LINE_AA,
        )
        mask = cv2.rotate(mask, cv2.ROTATE_180)

        x_start = origin[0]
        y_start = origin[1] - height
        x_end = x_start + mask.shape[1]
        y_end = y_start + mask.shape[0]

        frame_height, frame_width = frame.shape[:2]
        clipped_x_start = max(x_start, 0)
        clipped_y_start = max(y_start, 0)
        clipped_x_end = min(x_end, frame_width)
        clipped_y_end = min(y_end, frame_height)
        if clipped_x_start >= clipped_x_end or clipped_y_start >= clipped_y_end:
            return

        mask_x_start = clipped_x_start - x_start
        mask_y_start = clipped_y_start - y_start
        mask_x_end = mask_x_start + clipped_x_end - clipped_x_start
        mask_y_end = mask_y_start + clipped_y_end - clipped_y_start
        clipped_mask = mask[mask_y_start:mask_y_end, mask_x_start:mask_x_end]
        alpha = clipped_mask.astype(np.float32)[:, :, None] / 255.0
        region = frame[
            clipped_y_start:clipped_y_end,
            clipped_x_start:clipped_x_end,
        ]
        colour = np.asarray(color, dtype=np.float32)
        region[:] = (
            region.astype(np.float32) * (1.0 - alpha) + colour * alpha
        ).astype(np.uint8)
