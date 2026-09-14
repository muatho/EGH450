#!/usr/bin/env python3

"""Draw timestamp-matched ArUco and YOLO results on the OAK RGB image.

This node is deliberately presentation-only. Detection, PnP estimation,
map-frame transformation and landmark persistence remain responsibilities of
the ArUco processing node. Keeping drawing here prevents navigation behaviour
from becoming coupled to a debug image.

The camera image remains the driving input. Each frame waits briefly for
detector results, then publishes even if either detector is disabled, delayed,
or unavailable. Visualization can therefore degrade without hiding the camera
stream or affecting machine-readable navigation data.
"""

from collections import OrderedDict
import json
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rospy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, CompressedImage
from vision_msgs.msg import Detection2DArray

from depthai_publisher.msg import ArucoDetectionArray


class VisionOverlay:
    """Render diagnostics without changing machine-readable results."""

    def __init__(self):
        """Create inputs for calibration and timestamp-matched observations."""
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.lock = threading.Lock()
        self.pending_images = OrderedDict()
        self.aruco_by_stamp = OrderedDict()
        self.yolo_by_stamp = OrderedDict()

        self.image_topic = rospy.get_param(
            "~image_topic",
            "/depthai_node/image/compressed",
        )
        self.aruco_topic = rospy.get_param(
            "~aruco_observations_topic",
            "/aruco/observations",
        )
        self.output_topic = rospy.get_param(
            "~output_topic",
            "/processed_aruco/image/compressed",
        )
        self.camera_info_topic = rospy.get_param(
            "~camera_info_topic",
            "/depthai_node/camera/camera_info",
        )
        self.axis_length = rospy.get_param("~aruco_axis_length", 0.10)
        self.enable_yolo = rospy.get_param("~enable_yolo", True)
        self.yolo_topic = rospy.get_param(
            "~yolo_topic",
            "/yolo/detections",
        )
        # The OAK may return neural-network results several hundred
        # milliseconds after the corresponding RGB frame. Keep enough source
        # frames to join exact timestamps, but retain a bounded fallback so a
        # failed detector can never stop the presentation image.
        self.overlay_delay = rospy.get_param("~overlay_delay", 0.50)
        self.cache_size = rospy.get_param("~cache_size", 30)
        self.yolo_labels = self.read_yolo_labels()

        self.overlay_publisher = rospy.Publisher(
            self.output_topic,
            CompressedImage,
            queue_size=1,
        )

        # Calibration is fixed for the 640 x 360 stream, so retaining its most
        # recent CameraInfo is sufficient.
        self.camera_info_subscriber = rospy.Subscriber(
            self.camera_info_topic,
            CameraInfo,
            self.camera_info_callback,
            queue_size=1,
        )
        self.image_subscriber = rospy.Subscriber(
            self.image_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
        )
        self.aruco_subscriber = rospy.Subscriber(
            self.aruco_topic,
            ArucoDetectionArray,
            self.aruco_callback,
            queue_size=1,
        )
        self.yolo_subscriber = None
        if self.enable_yolo:
            self.yolo_subscriber = rospy.Subscriber(
                self.yolo_topic,
                Detection2DArray,
                self.yolo_callback,
                queue_size=1,
            )

        # Detector callbacks may arrive after their image callback. The timer
        # gives them a bounded opportunity to join by exact source timestamp,
        # then publishes the image regardless of which results arrived.
        self.flush_timer = rospy.Timer(
            rospy.Duration(0.01),
            self.flush_ready_images,
        )

        rospy.loginfo(
            "Vision overlay reading %s; optional results wait %.3f s",
            self.image_topic,
            self.overlay_delay,
        )

    def read_yolo_labels(self):
        """Read display names from the same model JSON used by DepthAI."""
        package_dir = Path(__file__).resolve().parent
        default_config = (
            package_dir
            / "models"
            / "best_openvino_2022.1_6shave"
            / "best.json"
        )
        config_path = Path(rospy.get_param(
            "~yolo_config_path",
            str(default_config),
        )).expanduser()

        try:
            with config_path.open("r") as config_file:
                labels = json.load(config_file)["mappings"]["labels"]
        except (OSError, KeyError, TypeError, ValueError) as error:
            rospy.logwarn(
                "Could not read YOLO display labels from %s: %s; showing IDs",
                config_path,
                error,
            )
            return []

        return [str(label) for label in labels]

    def camera_info_callback(self, message):
        """Retain K and D used by PnP so axes project onto the same pixels."""
        self.camera_matrix = np.asarray(message.K, dtype=np.float64).reshape(3, 3)
        self.distortion_coefficients = np.asarray(
            message.D,
            dtype=np.float64,
        )

    @staticmethod
    def stamp_key(message):
        """Convert a ROS header timestamp into an exact dictionary key."""
        return message.header.stamp.to_nsec()

    def store_bounded(self, cache, key, value):
        """Store newest timestamped data while bounding diagnostic memory."""
        cache[key] = value
        cache.move_to_end(key)
        while len(cache) > self.cache_size:
            cache.popitem(last=False)

    def image_callback(self, message):
        """Queue a camera frame with a wall-clock publication deadline."""
        key = self.stamp_key(message)
        with self.lock:
            self.store_bounded(
                self.pending_images,
                key,
                (message, time.monotonic()),
            )

    def aruco_callback(self, message):
        """Cache ArUco results under their exact source-image timestamp."""
        key = self.stamp_key(message)
        with self.lock:
            self.store_bounded(self.aruco_by_stamp, key, message)

    def yolo_callback(self, message):
        """Cache YOLO results under their exact source-image timestamp."""
        key = self.stamp_key(message)
        with self.lock:
            self.store_bounded(self.yolo_by_stamp, key, message)

    def flush_ready_images(self, _event):
        """Publish every image whose detector-wait deadline has elapsed."""
        now = time.monotonic()
        ready = []

        with self.lock:
            for key, (image, arrival_time) in list(self.pending_images.items()):
                aruco_ready = key in self.aruco_by_stamp
                yolo_ready = (
                    not self.enable_yolo
                    or key in self.yolo_by_stamp
                )
                deadline_reached = now - arrival_time >= self.overlay_delay

                # Publish immediately when every enabled detector has answered
                # for this exact image. Otherwise publish at the deadline so a
                # missing or failed detector cannot hide the camera stream.
                if not (aruco_ready and yolo_ready) and not deadline_reached:
                    continue
                ready.append((
                    image,
                    self.aruco_by_stamp.pop(key, None),
                    self.yolo_by_stamp.pop(key, None),
                ))
                del self.pending_images[key]

        for image, aruco, yolo in ready:
            self.publish_overlay(image, aruco, yolo)

    def publish_overlay(self, image_message, aruco_array, yolo_array):
        """Draw available results and preserve the source image header."""
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(image_message)
        except CvBridgeError as error:
            rospy.logerr("Could not decode overlay source image: %s", error)
            return

        if aruco_array is not None:
            for observation in aruco_array.detections:
                self.draw_aruco_observation(frame, observation)

        if yolo_array is not None:
            for detection in yolo_array.detections:
                self.draw_yolo_detection(frame, detection)

        success, encoded = cv2.imencode(".jpg", frame)
        if not success:
            rospy.logwarn("Failed to JPEG-encode vision overlay")
            return

        output = CompressedImage()
        output.header = image_message.header
        output.format = "jpeg"
        output.data = encoded.tobytes()
        self.overlay_publisher.publish(output)

    def draw_yolo_detection(self, frame, detection):
        """Draw one YOLO box, configured class name and confidence score."""
        centre_x = detection.bbox.center.x
        centre_y = detection.bbox.center.y
        half_width = detection.bbox.size_x / 2.0
        half_height = detection.bbox.size_y / 2.0
        frame_height, frame_width = frame.shape[:2]

        top_left = (
            max(0, int(round(centre_x - half_width))),
            max(0, int(round(centre_y - half_height))),
        )
        bottom_right = (
            min(frame_width - 1, int(round(centre_x + half_width))),
            min(frame_height - 1, int(round(centre_y + half_height))),
        )
        if bottom_right[0] <= top_left[0] or bottom_right[1] <= top_left[1]:
            return

        cv2.rectangle(frame, top_left, bottom_right, (255, 0, 0), 2)

        if detection.results:
            hypothesis = detection.results[0]
            class_id = int(hypothesis.id)
            if 0 <= class_id < len(self.yolo_labels):
                class_name = self.yolo_labels[class_id]
            else:
                class_name = "class {}".format(class_id)
            label = "{} {:.0f}%".format(
                class_name,
                100.0 * hypothesis.score,
            )
        else:
            label = "object"

        self.draw_text_180(
            frame,
            label,
            (top_left[0], max(top_left[1] - 5, 15)),
            cv2.FONT_HERSHEY_COMPLEX,
            0.5,
            (255, 0, 0),
            2,
        )

    def draw_aruco_observation(self, frame, observation):
        """Draw one marker's corners, ID and optional accepted PnP result."""
        if len(observation.corners) != 4:
            rospy.logwarn_throttle(
                5.0,
                "Ignoring ArUco observation %s with %d corners",
                observation.id,
                len(observation.corners),
            )
            return

        corners = [
            (int(round(point.x)), int(round(point.y)))
            for point in observation.corners
        ]
        top_left, top_right, bottom_right, bottom_left = corners

        cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
        cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
        cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
        cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)

        self.draw_text_180(
            frame,
            str(observation.id),
            (top_left[0], top_right[1] - 15),
            cv2.FONT_HERSHEY_COMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )

        if not observation.pose_valid:
            return
        if self.camera_matrix is None or self.distortion_coefficients is None:
            rospy.logwarn_throttle(
                5.0,
                "CameraInfo unavailable: drawing ArUco borders without axes",
            )
            return

        rvec = np.array([
            observation.rvec.x,
            observation.rvec.y,
            observation.rvec.z,
        ], dtype=np.float64).reshape(3, 1)
        tvec = np.array([
            observation.tvec.x,
            observation.tvec.y,
            observation.tvec.z,
        ], dtype=np.float64).reshape(3, 1)

        cv2.drawFrameAxes(
            frame,
            self.camera_matrix,
            self.distortion_coefficients,
            rvec,
            tvec,
            self.axis_length,
        )

        pose_lines = (
            "err: {:.2f}px".format(observation.reprojection_error),
            "Z: {:.2f}".format(tvec[2, 0]),
            "Y: {:.2f}".format(tvec[1, 0]),
            "X: {:.2f}".format(tvec[0, 0]),
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
            text,
            font,
            scale,
            thickness,
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

        clipped_mask = mask[
            mask_y_start:mask_y_end,
            mask_x_start:mask_x_end,
        ]
        region = frame[
            clipped_y_start:clipped_y_end,
            clipped_x_start:clipped_x_end,
        ]
        region[clipped_mask > 0] = color


def main():
    """Start the presentation-only overlay node."""
    rospy.init_node("vision_overlay")
    VisionOverlay()
    rospy.spin()


if __name__ == "__main__":
    main()
