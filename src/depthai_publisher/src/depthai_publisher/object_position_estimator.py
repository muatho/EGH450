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
from dataclasses import dataclass


from depthai_publisher.camera_to_map import CameraToMap
from depthai_publisher.msg import (
    TargetObservation,
    TargetObservationArray,
)



@dataclass(frozen=True)
class DepthEstimate:
    """
    Robust depth result calculated inside one detection box.

    This is an internal Python value, not a ROS message. It keeps the map-point
    calculation and its supporting quality measurements together.
    """

    camera_point: np.ndarray
    median_metres: float
    mad_metres: float
    valid_samples: int
    valid_fraction: float


class ObjectPositionEstimator:
    """Publish independent 3D observations without tracking or persistence."""

    def __init__(self):
        """Create synchronized perception inputs and independent pose state."""
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.camera_to_map = CameraToMap()
        self.pose_lock = threading.RLock()
        self.pose_history = deque(maxlen=500)

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
            "~target_output_topic",
            "/objects/target_observations",
        )


        # Use the central half of each box to reduce contamination by the
        # background immediately outside an imperfect detector boundary.
        self.inner_box_scale = rospy.get_param("~inner_box_scale", 0.50)
        self.minimum_depth = rospy.get_param("~minimum_depth", 0.20)
        self.maximum_depth = rospy.get_param("~maximum_depth", 10.0)
        self.minimum_depth_samples = rospy.get_param(
            "~minimum_depth_samples", 20
        )
        self.max_pose_age = rospy.get_param("~max_pose_age", 0.20)
        self.pose_history_duration = rospy.get_param("~pose_history_duration", 2.0)

        if self.max_pose_age <= 0:
            raise ValueError("~max_pose_age must be greater than zero")

        if self.pose_history_duration <= self.max_pose_age:
            raise ValueError("~pose_history_duration must exceed ~max_pose_age")

        self.output_publisher = rospy.Publisher(
            self.output_topic,
            Detection3DArray,
            queue_size=1,
        )
        # Retain Detection3DArray for existing consumers while publishing the
        # richer observation required by target evidence and centring.
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
        """Retain enough timestamped poses to cover perception latency."""
        with self.pose_lock:
            self.pose_history.append(message)
            newest_stamp = message.header.stamp

            while len(self.pose_history) > 1:
                oldest_age = (newest_stamp - self.pose_history[0].header.stamp).to_sec()

                if oldest_age <= self.pose_history_duration:
                    break

                self.pose_history.popleft()

    def pose_for_stamp(self, source_stamp):
        """Return the buffered pose nearest the source-image timestamp."""
        with self.pose_lock:
            poses = tuple(self.pose_history)

        if not poses:
            return None

        pose = min(
            poses,
            key=lambda candidate: abs((source_stamp - candidate.header.stamp).to_sec()),
        )
        pose_age = abs((source_stamp - pose.header.stamp).to_sec())
        return pose if pose_age <= self.max_pose_age else None

    def publish_outputs(
        self,
        detection_output,
        target_output,
    ):
        """Publish compatible legacy and rich outputs from the same frame."""

        self.output_publisher.publish(
            detection_output
        )
        self.target_output_publisher.publish(
            target_output
        )

    def perception_callback(self, detections, depth_message):
        """Transform all valid observations from one synchronized frame."""
        output = Detection3DArray()
        output.header.stamp = detections.header.stamp
        output.header.frame_id = "map"

        target_output = TargetObservationArray()

        # Retain the source-image header. Its timestamp, frame ID and sequence
        # identify the exact image that produced every observation.
        target_output.header = detections.header

        if self.camera_matrix is None:
            rospy.logwarn_throttle(
                5.0, "CameraInfo unavailable: no global object observations"
            )
            self.publish_outputs(output,target_output,)
            return

        pose = self.pose_for_stamp(detections.header.stamp)

        if pose is None:
            rospy.logwarn_throttle(
                5.0,
                "No vehicle pose close to the source-image timestamp: no global object observations",
            )
            self.publish_outputs(output,target_output,)
            return

        try:
            depth_mm = self.bridge.imgmsg_to_cv2(
                depth_message, desired_encoding="16UC1"
            )
        except CvBridgeError as error:
            rospy.logerr("Could not decode aligned depth image: %s", error)
            self.publish_outputs(output,target_output,)
            return

        for detection_2d in detections.detections:
            if not detection_2d.results:
                continue
            depth_estimate = self.estimate_camera_point(
                depth_mm,
                detection_2d,
            )

            if depth_estimate is None:
                continue

            map_point = self.camera_to_map.camera_point_to_map(
                depth_estimate.camera_point,
                pose,
            )

            if map_point is None:
                continue
            output.detections.append(
                self.build_detection_3d(
                    detection_2d,
                    map_point,
                    output.header,
                )
            )

            target_output.observations.append(
                self.build_target_observation(
                    detection_2d=detection_2d,
                    map_point=map_point,
                    depth_estimate=depth_estimate,
                    image_width=depth_mm.shape[1],
                    image_height=depth_mm.shape[0],
                )
            )

        self.publish_outputs(output,target_output)

    def estimate_camera_point(
        self,
        depth_mm,
        detection,
    ):
        """
        Return robust optical XYZ and supporting depth-quality measurements.

        The central portion of the box reduces background contamination.
        Coordinate-wise medians provide the camera-frame point, while median
        absolute deviation describes the spread of the contributing depths.
        """

        image_height, image_width = depth_mm.shape[:2]
        centre_x = float(detection.bbox.center.x)
        centre_y = float(detection.bbox.center.y)

        half_width = (
            0.5
            * float(detection.bbox.size_x)
            * self.inner_box_scale
        )
        half_height = (
            0.5
            * float(detection.bbox.size_y)
            * self.inner_box_scale
        )

        x_min = max(
            0,
            int(np.floor(centre_x - half_width)),
        )
        x_max = min(
            image_width,
            int(np.ceil(centre_x + half_width)),
        )
        y_min = max(
            0,
            int(np.floor(centre_y - half_height)),
        )
        y_max = min(
            image_height,
            int(np.ceil(centre_y + half_height)),
        )

        if x_max <= x_min or y_max <= y_min:
            return None

        roi = depth_mm[
            y_min:y_max,
            x_min:x_max,
        ]

        valid = (
            (roi >= self.minimum_depth * 1000.0)
            & (roi <= self.maximum_depth * 1000.0)
        )

        rows, columns = np.nonzero(valid)
        valid_sample_count = int(rows.size)

        if valid_sample_count < self.minimum_depth_samples:
            return None

        pixels = np.column_stack(
            (
                columns + x_min,
                rows + y_min,
            )
        ).astype(np.float64)

        normalized = cv2.undistortPoints(
            pixels.reshape(-1, 1, 2),
            self.camera_matrix,
            self.distortion_coefficients,
        ).reshape(-1, 2)

        z_metres = (
            roi[valid].astype(np.float64) / 1000.0
        )

        camera_points = np.column_stack(
            (
                normalized[:, 0] * z_metres,
                normalized[:, 1] * z_metres,
                z_metres,
            )
        )

        median_depth = float(np.median(z_metres))
        depth_mad = float(
            np.median(
                np.abs(z_metres - median_depth)
            )
        )

        return DepthEstimate(
            camera_point=np.median(
                camera_points,
                axis=0,
            ),
            median_metres=median_depth,
            mad_metres=depth_mad,
            valid_samples=valid_sample_count,
            valid_fraction=(
                valid_sample_count / float(roi.size)
            ),
        )

    @staticmethod
    def build_target_observation(
        detection_2d,
        map_point,
        depth_estimate,
        image_width,
        image_height,
    ):
        """
        Combine one raw YOLO result with its spatial and depth evidence.

        No temporal confirmation, tracking or class modification is applied.
        """

        source_hypothesis = detection_2d.results[0]

        observation = TargetObservation()
        observation.class_id = int(
            source_hypothesis.id
        )
        observation.confidence = float(
            source_hypothesis.score
        )

        observation.bbox_center_x = float(
            detection_2d.bbox.center.x
        )
        observation.bbox_center_y = float(
            detection_2d.bbox.center.y
        )
        observation.bbox_width = float(
            detection_2d.bbox.size_x
        )
        observation.bbox_height = float(
            detection_2d.bbox.size_y
        )
        observation.image_width = int(
            image_width
        )
        observation.image_height = int(
            image_height
        )

        observation.map_position.x = float(
            map_point[0]
        )
        observation.map_position.y = float(
            map_point[1]
        )
        observation.map_position.z = float(
            map_point[2]
        )

        observation.depth_median_metres = (
            depth_estimate.median_metres
        )
        observation.depth_mad_metres = (
            depth_estimate.mad_metres
        )
        observation.valid_depth_samples = (
            depth_estimate.valid_samples
        )
        observation.valid_depth_fraction = (
            depth_estimate.valid_fraction
        )

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
