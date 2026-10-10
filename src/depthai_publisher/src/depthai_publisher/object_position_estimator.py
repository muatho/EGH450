#!/usr/bin/env python3

"""Estimate transient map-frame object positions from YOLO and depth."""

import cv2
import message_filters
import numpy as np
import rospy
import threading
from collections import deque


from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import (
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

from depthai_publisher.camera_to_map import CameraToMap
from depthai_publisher.msg import TargetObservation, TargetObservationArray


class ObjectPositionEstimator:
    """Publish independent 3D observations without tracking or persistence."""

    DEPTH_PATCH_SIZE = 7

    def __init__(self):
        """Create synchronized perception inputs and independent pose state."""
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion_coefficients = None

        # Retain recent aircraft poses so each detection can use the pose recorded
        # closest to the source image rather than the newest pose available later.
        self.pose_history = deque(maxlen=200)
        self.pose_lock = threading.Lock()
        self.camera_to_map = CameraToMap()

        self.detection_topic = rospy.get_param(
            "~detection_topic", "/yolo/detections"
        )
        self.depth_topic = rospy.get_param(
            "~depth_topic", "/depthai_node/depth/aligned"
        )
        self.pose_topic = rospy.get_param(
            "~pose_topic", "/mavros/local_position/pose"
        )
        self.output_topic = rospy.get_param(
            "~output_topic", "/objects/observations"
        )
        self.target_output_topic = rospy.get_param(
            "~target_output_topic", "/objects/target_observations"
        )

        self.minimum_depth = rospy.get_param("~minimum_depth", 0.20)
        self.maximum_depth = rospy.get_param("~maximum_depth", 10.0)
        self.minimum_depth_samples = rospy.get_param(
            "~minimum_depth_samples", 20
        )
        self.max_pose_age = rospy.get_param("~max_pose_age", 0.20)

        self.output_publisher = rospy.Publisher(
            self.output_topic,
            Detection3DArray,
            queue_size=1,
        )
        self.target_output_publisher = rospy.Publisher(
            self.target_output_topic,
            TargetObservationArray,
            queue_size=1,
        )
        self.camera_info_subscriber = rospy.Subscriber(
            "/depthai_node/camera/camera_info",
            CameraInfo,
            self.camera_info_callback,
            queue_size=1,
        )
        self.pose_subscriber = rospy.Subscriber(
            self.pose_topic,
            PoseStamped,
            self.pose_callback,
            queue_size=1,
        )

        detection_subscriber = message_filters.Subscriber(
            self.detection_topic,
            Detection2DArray,
            queue_size=1,
        )
        depth_subscriber = message_filters.Subscriber(
            self.depth_topic,
            Image,
            queue_size=1,
        )
        # dai_publisher gives paired RGB/depth/YOLO data the same timestamp.
        # Exact synchronization prevents any reuse of stale depth.
        self.synchronizer = message_filters.TimeSynchronizer(
            [detection_subscriber, depth_subscriber],
            queue_size=10,
        )
        self.synchronizer.registerCallback(self.perception_callback)
        # Keep references alive for the lifetime of this node.
        self.detection_subscriber = detection_subscriber
        self.depth_subscriber = depth_subscriber

        rospy.loginfo(
            "Object position estimator: %s + %s -> %s and %s",
            self.detection_topic,
            self.depth_topic,
            self.output_topic,
            self.target_output_topic,
        )

    def camera_info_callback(self, message):
        """Retain calibration for distortion-aware pixel deprojection."""
        self.camera_matrix = np.asarray(message.K, dtype=np.float64).reshape(3, 3)
        self.distortion_coefficients = np.asarray(
            message.D, dtype=np.float64
        )

    def pose_callback(self, message):
        """Store recent aircraft poses for timestamp-aligned lookup."""

        with self.pose_lock:
            self.pose_history.append(message)

    def pose_for_stamp(self, source_stamp):
        """Return the aircraft pose closest to the source-image timestamp."""

        with self.pose_lock:
            poses = tuple(self.pose_history)

        if not poses:
            return None

        pose = min(
            poses,
            key=lambda candidate: abs(
                (source_stamp - candidate.header.stamp).to_sec()
            ),
        )
        pose_age = abs((source_stamp - pose.header.stamp).to_sec())

        if pose_age > self.max_pose_age:
            return None

        return pose

    def publish_outputs(self, detection_output, target_output):
        """Publish both observation formats for one source image."""

        self.output_publisher.publish(detection_output)
        self.target_output_publisher.publish(target_output)

    def perception_callback(self, detections, depth_message):
        """Combine one detection frame with aligned depth and aircraft pose."""

        detection_output = Detection3DArray()
        detection_output.header.stamp = detections.header.stamp
        detection_output.header.frame_id = "map"

        target_output = TargetObservationArray()
        target_output.header = detections.header
        target_output.map_frame_id = detection_output.header.frame_id
        target_output.image_width = depth_message.width
        target_output.image_height = depth_message.height

        observation_pairs = []

        for detection_2d in detections.detections:
            if not detection_2d.results:
                continue

            target_observation = self.build_target_observation(detection_2d)
            target_output.observations.append(target_observation)
            observation_pairs.append((detection_2d, target_observation))

        if self.camera_matrix is None:
            rospy.logwarn_throttle(
                5.0,
                "CameraInfo unavailable: target map positions are invalid",
            )
            self.publish_outputs(detection_output, target_output)
            return

        pose = self.pose_for_stamp(detections.header.stamp)

        if pose is None:
            rospy.logwarn_throttle(
                5.0,
                "No aircraft pose near the image timestamp: target map positions are invalid",
            )
            self.publish_outputs(detection_output, target_output)
            return

        try:
            depth_mm = self.bridge.imgmsg_to_cv2(
                depth_message,
                desired_encoding="16UC1",
            )
        except CvBridgeError as error:
            rospy.logerr("Could not decode aligned depth image: %s", error)
            self.publish_outputs(detection_output, target_output)
            return

        for detection_2d, target_observation in observation_pairs:
            camera_point = self.estimate_camera_point(
                depth_mm,
                detection_2d,
            )
            if camera_point is None:
                continue

            map_point = self.camera_to_map.camera_point_to_map(
                camera_point,
                pose,
            )
            if map_point is None:
                continue

            target_observation.map_position_valid = True
            target_observation.map_position.x = float(map_point[0])
            target_observation.map_position.y = float(map_point[1])
            target_observation.map_position.z = float(map_point[2])

            detection_output.detections.append(
                self.build_detection_3d(
                    detection_2d,
                    map_point,
                    detection_output.header,
                )
            )

        self.publish_outputs(detection_output, target_output)

    def estimate_camera_point(self, depth_mm, detection):
        """Estimate optical XYZ from depth near the detection centre."""

        image_height, image_width = depth_mm.shape[:2]
        centre_x = float(detection.bbox.center.x)
        centre_y = float(detection.bbox.center.y)

        centre_column = int(round(centre_x))
        centre_row = int(round(centre_y))
        half_patch = self.DEPTH_PATCH_SIZE // 2

        x_min = max(
            0,
            min(
                centre_column - half_patch,
                image_width - self.DEPTH_PATCH_SIZE,
            ),
        )
        y_min = max(
            0,
            min(
                centre_row - half_patch,
                image_height - self.DEPTH_PATCH_SIZE,
            ),
        )
        x_max = x_min + self.DEPTH_PATCH_SIZE
        y_max = y_min + self.DEPTH_PATCH_SIZE

        depth_patch = depth_mm[y_min:y_max, x_min:x_max]
        valid_depths = depth_patch[
            (depth_patch >= self.minimum_depth * 1000.0)
            & (depth_patch <= self.maximum_depth * 1000.0)
        ]

        if valid_depths.size < self.minimum_depth_samples:
            return None

        median_depth = float(np.median(valid_depths)) / 1000.0

        centre_pixel = np.array(
            [[[centre_x, centre_y]]],
            dtype=np.float64,
        )
        normalized_ray = cv2.undistortPoints(
            centre_pixel,
            self.camera_matrix,
            self.distortion_coefficients,
        )[0, 0]

        return np.array([
            normalized_ray[0] * median_depth,
            normalized_ray[1] * median_depth,
            median_depth,
        ])

    @staticmethod
    def build_target_observation(detection_2d):
        """Copy one raw YOLO result into a target observation."""

        result = detection_2d.results[0]

        observation = TargetObservation()
        observation.class_id = int(result.id)
        observation.confidence = float(result.score)
        observation.bbox_center_x = float(detection_2d.bbox.center.x)
        observation.bbox_center_y = float(detection_2d.bbox.center.y)
        observation.bbox_width = float(detection_2d.bbox.size_x)
        observation.bbox_height = float(detection_2d.bbox.size_y)
        observation.map_position_valid = False

        return observation

    @staticmethod
    def build_detection_3d(detection_2d, map_point, header):
        """Copy class evidence into one map-frame point observation."""
        source_hypothesis = detection_2d.results[0]
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.id = source_hypothesis.id
        hypothesis.score = source_hypothesis.score
        hypothesis.pose.pose.position.x = float(map_point[0])
        hypothesis.pose.pose.position.y = float(map_point[1])
        hypothesis.pose.pose.position.z = float(map_point[2])
        hypothesis.pose.pose.orientation.w = 1.0

        detection_3d = Detection3D()
        detection_3d.header = header
        detection_3d.results.append(hypothesis)
        # No physical 3D extent or orientation has been estimated. The box
        # centre therefore represents the measured position only.
        detection_3d.bbox.center.position = hypothesis.pose.pose.position
        detection_3d.bbox.center.orientation.w = 1.0
        return detection_3d


def main():
    """Start the stateless object-position estimator."""
    rospy.init_node("object_position_estimator")
    ObjectPositionEstimator()
    rospy.spin()


if __name__ == "__main__":
    main()
