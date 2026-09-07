#!/usr/bin/env python3

"""Offline synthetic checks for ArucoPoseEstimator's IPPE-square solver."""

import cv2
import numpy as np

from depthai_publisher.aruco_pose_estimator_PnP import ArucoPoseEstimator


def rotation_difference_degrees(first_rvec, second_rvec):
    """Return the smallest angle separating two Rodrigues rotations."""
    first_rotation, _ = cv2.Rodrigues(first_rvec)
    second_rotation, _ = cv2.Rodrigues(second_rvec)
    relative_rotation = first_rotation @ second_rotation.T
    cosine = np.clip((np.trace(relative_rotation) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.degrees(np.arccos(cosine)))


def estimate_from_projected_corners(estimator, rvec, tvec, noise_pixels=0.0):
    """Project a known pose, optionally perturb pixels, then estimate it back."""
    corners, _ = cv2.projectPoints(
        estimator.obj_points,
        rvec,
        tvec,
        estimator.camera_matrix,
        estimator.dist_coeffs,
    )
    corners = corners.reshape((4, 2))

    if noise_pixels > 0.0:
        random = np.random.default_rng(seed=450)
        corners += random.normal(0.0, noise_pixels, corners.shape)

    marker_corner = corners.astype(np.float32).reshape((1, 4, 2))
    return estimator.estimate_pose(marker_corner)


def main():
    """Run exact, noisy, and invalid-corner tests without ROS or a camera."""
    estimator = ArucoPoseEstimator(
        marker_size=0.200,
        max_reprojection_error=3.0,
    )

    # A simple, undistorted 640 x 360 test camera. These are synthetic values;
    # the test verifies solver logic rather than the physical OAK calibration.
    estimator.camera_matrix = np.array([
        [512.0, 0.0, 320.0],
        [0.0, 512.0, 180.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)
    estimator.dist_coeffs = np.zeros(8, dtype=np.float64)
    estimator.got_camera_info = True

    # Near-pi X rotation faces the marker toward the optical camera, with a
    # modest tilt to avoid the perfectly front-on degenerate case.
    expected_rvec = np.array([[2.90], [0.15], [-0.10]], dtype=np.float64)
    expected_tvec = np.array([[0.20], [-0.10], [2.00]], dtype=np.float64)

    success, exact_rvec, exact_tvec = estimate_from_projected_corners(
        estimator,
        expected_rvec,
        expected_tvec,
    )
    assert success, "Exact projected corners should produce a valid pose"

    exact_translation_error = float(np.linalg.norm(exact_tvec - expected_tvec))
    exact_rotation_error = rotation_difference_degrees(exact_rvec, expected_rvec)
    assert exact_translation_error < 1e-4
    assert exact_rotation_error < 1e-2

    print("PASS exact projection")
    print(f"  translation error: {exact_translation_error:.8f} m")
    print(f"  rotation error:    {exact_rotation_error:.8f} deg")
    print(f"  reprojection RMS:  {estimator.last_reprojection_error:.8f} px")

    success, noisy_rvec, noisy_tvec = estimate_from_projected_corners(
        estimator,
        expected_rvec,
        expected_tvec,
        noise_pixels=0.25,
    )
    assert success, "Small subpixel corner noise should still produce a pose"

    noisy_translation_error = float(np.linalg.norm(noisy_tvec - expected_tvec))
    noisy_rotation_error = rotation_difference_degrees(noisy_rvec, expected_rvec)
    assert noisy_translation_error < 0.10
    assert noisy_rotation_error < 10.0

    print("PASS noisy projection (0.25 px standard deviation)")
    print(f"  translation error: {noisy_translation_error:.6f} m")
    print(f"  rotation error:    {noisy_rotation_error:.6f} deg")
    print(f"  reprojection RMS:  {estimator.last_reprojection_error:.6f} px")

    # One corner displaced by 30 pixels cannot be explained by a rigid square
    # under the configured 3 px RMS limit and must be rejected.
    bad_corners, _ = cv2.projectPoints(
        estimator.obj_points,
        expected_rvec,
        expected_tvec,
        estimator.camera_matrix,
        estimator.dist_coeffs,
    )
    bad_corners = bad_corners.reshape((4, 2))
    bad_corners[0] += np.array([30.0, -30.0])
    success, _, _ = estimator.estimate_pose(
        bad_corners.astype(np.float32).reshape((1, 4, 2))
    )
    assert not success, "Grossly inconsistent corners should be rejected"

    print("PASS grossly inconsistent corner rejection")
    print("All synthetic IPPE-square tests passed.")


if __name__ == "__main__":
    main()
