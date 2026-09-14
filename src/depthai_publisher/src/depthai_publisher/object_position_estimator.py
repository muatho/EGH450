#!/usr/bin/env python3

"""Estimate transient map-frame object positions from YOLO and depth."""

import cv2
import message_filters
import numpy as np
import rospy

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


class ObjectPositionEstimator:
    """Publish independent 3D observations without tracking or persistence."""

    def __init__(self):
        """Create synchronized perception inputs and independent pose state."""
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.latest_pose = None
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

        # Use the central half of each box to reduce contamination by the
        # background immediately outside an imperfect detector boundary.
        self.inner_box_scale = rospy.get_param("~inner_box_scale", 0.50)
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
            "Object position estimator: %s + %s -> %s",
            self.detection_topic,
            self.depth_topic,
            self.output_topic,
        )

    def camera_info_callback(self, message):
        """Retain calibration for distortion-aware pixel deprojection."""
        self.camera_matrix = np.asarray(message.K, dtype=np.float64).reshape(3, 3)
        self.distortion_coefficients = np.asarray(
            message.D, dtype=np.float64
        )

    def pose_callback(self, message):
        """Retain the newest vehicle pose for an explicit age check."""
        self.latest_pose = message

    def perception_callback(self, detections, depth_message):
        """Transform all valid observations from one synchronized frame."""
        output = Detection3DArray()
        output.header.stamp = detections.header.stamp
        output.header.frame_id = "map"

        if self.camera_matrix is None:
            rospy.logwarn_throttle(
                5.0, "CameraInfo unavailable: no global object observations"
            )
            self.output_publisher.publish(output)
            return

        pose = self.latest_pose
        if pose is not None:
            pose_age = abs(
                (detections.header.stamp - pose.header.stamp).to_sec()
            )
            if pose_age > self.max_pose_age:
                pose = None
        if pose is None:
            rospy.logwarn_throttle(
                5.0,
                "No recent vehicle pose: no global object observations",
            )
            self.output_publisher.publish(output)
            return

        try:
            depth_mm = self.bridge.imgmsg_to_cv2(
                depth_message, desired_encoding="16UC1"
            )
        except CvBridgeError as error:
            rospy.logerr("Could not decode aligned depth image: %s", error)
            self.output_publisher.publish(output)
            return

        for detection_2d in detections.detections:
            if not detection_2d.results:
                continue
            camera_point = self.estimate_camera_point(depth_mm, detection_2d)
            if camera_point is None:
                continue
            map_point = self.camera_to_map.camera_point_to_map(
                camera_point, pose
            )
            if map_point is None:
                continue
            output.detections.append(
                self.build_detection_3d(detection_2d, map_point, output.header)
            )

        self.output_publisher.publish(output)

    def estimate_camera_point(self, depth_mm, detection):
        """Return robust optical XYZ from valid pixels inside one inner box."""
        image_height, image_width = depth_mm.shape[:2]
        centre_x = float(detection.bbox.center.x)
        centre_y = float(detection.bbox.center.y)
        half_width = 0.5 * float(detection.bbox.size_x) * self.inner_box_scale
        half_height = 0.5 * float(detection.bbox.size_y) * self.inner_box_scale

        x_min = max(0, int(np.floor(centre_x - half_width)))
        x_max = min(image_width, int(np.ceil(centre_x + half_width)))
        y_min = max(0, int(np.floor(centre_y - half_height)))
        y_max = min(image_height, int(np.ceil(centre_y + half_height)))
        if x_max <= x_min or y_max <= y_min:
            return None

        roi = depth_mm[y_min:y_max, x_min:x_max]
        valid = (
            (roi >= self.minimum_depth * 1000.0)
            & (roi <= self.maximum_depth * 1000.0)
        )
        rows, columns = np.nonzero(valid)
        if rows.size < self.minimum_depth_samples:
            return None

        # Deproject every valid pixel, then take coordinate-wise medians. This
        # is more robust than trusting one centre pixel and preserves the
        # rational distortion model used by the RGB stream.
        pixels = np.column_stack((columns + x_min, rows + y_min)).astype(
            np.float64
        )
        normalized = cv2.undistortPoints(
            pixels.reshape(-1, 1, 2),
            self.camera_matrix,
            self.distortion_coefficients,
        ).reshape(-1, 2)
        z_metres = roi[valid].astype(np.float64) / 1000.0
        camera_points = np.column_stack((
            normalized[:, 0] * z_metres,
            normalized[:, 1] * z_metres,
            z_metres,
        ))
        return np.median(camera_points, axis=0)

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
