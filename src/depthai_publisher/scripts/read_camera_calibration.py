#!/usr/bin/env python3

"""Read and print calibration stored on a connected OAK camera."""

import argparse

import depthai as dai
import numpy as np


def format_matrix(matrix):
    """Return a readable string representation of a numeric matrix."""
    return np.array2string(
        np.asarray(matrix, dtype=float),
        precision=6,
        suppress_small=True,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Read RGB calibration from the connected OAK camera EEPROM."
    )
    parser.add_argument(
        "--width",
        type=int,
        help="Optional output-image width for adjusted intrinsics.",
    )
    parser.add_argument(
        "--height",
        type=int,
        help="Optional output-image height for adjusted intrinsics.",
    )
    args = parser.parse_args()

    if (args.width is None) != (args.height is None):
        parser.error("--width and --height must be supplied together")

    # RGB is the name used by older DepthAI releases; CAM_A is its newer name.
    rgb_socket = (
        dai.CameraBoardSocket.CAM_A
        if hasattr(dai.CameraBoardSocket, "CAM_A")
        else dai.CameraBoardSocket.RGB
    )

    with dai.Device() as device:
        calibration = device.readCalibration()
        device_info = device.getDeviceInfo()

        default_k, default_width, default_height = (
            calibration.getDefaultIntrinsics(rgb_socket)
        )
        distortion = calibration.getDistortionCoefficients(rgb_socket)

        print("Connected OAK camera")
        print(f"  MX ID: {device_info.getMxId()}")
        print(f"  RGB socket: {rgb_socket}")
        print(f"  Stored calibration size: {default_width} x {default_height}")
        print("\nStored/default intrinsic matrix K:")
        print(format_matrix(default_k))
        print("\nStored distortion coefficients D:")
        print(format_matrix(distortion))

        if args.width is not None:
            adjusted_k = calibration.getCameraIntrinsics(
                rgb_socket,
                args.width,
                args.height,
            )
            print(
                f"\nIntrinsic matrix K adjusted for "
                f"{args.width} x {args.height}:"
            )
            print(format_matrix(adjusted_k))


if __name__ == "__main__":
    main()
