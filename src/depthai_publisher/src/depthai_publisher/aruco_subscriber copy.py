#!/usr/bin/env python3

import cv2

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R

from depthai_publisher.aruco_pose_estimator_PnP import ArucoPoseEstimator
from depthai_publisher.aruco_global_transform import ArucoGlobalTransformer

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class ArucoDetector():
    """Detect, locate, annotate, and publish ArUco marker observations.

    Every compressed OAK image is detected and republished independently.
    MAVROS pose is optional: when a sufficiently recent pose is available it
    is used to transform marker observations into the map frame.

    CameraInfo is a separate calibration input. Its K and D values describe the
    fixed 640 x 360 stream, so the most recently received calibration can be
    retained by ArucoPoseEstimator rather than synchronized for every frame.
    """

    # The printed markers must use this same dictionary family and ID range.
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    aruco_params = cv2.aruco.DetectorParameters()
    # Refine initially detected corners to subpixel precision. PnP is highly
    # sensitive to corner noise because all pose information comes from only
    # four points, especially when a distant marker occupies few pixels.
    aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    # Keeping external interfaces together makes the node's ROS connections
    # visible without searching through the processing functions.
    frame_sub_topic = '/depthai_node/image/compressed'
    pose_sub_topic = '/mavros/local_position/pose'

    def __init__(self):
        """Create publishers and independent image, pose, and info inputs."""
        # Human-readable JPEG containing borders, IDs, axes, and PnP values.
        self.aruco_pub = rospy.Publisher(
            '/processed_aruco/image/compressed', CompressedImage, queue_size=1)

        # Machine/RViz-readable global marker locations in the map frame.
        self.marker_pub = rospy.Publisher(
                    "/world_model/aruco_markers",
                    MarkerArray,
                    queue_size=1,
                    # The array represents persistent world knowledge. Send
                    # its latest value immediately to subscribers that connect
                    # or restart after the most recent marker observation.
                    latch=True,
                )

        # CvBridge decodes ROS CompressedImage data into an OpenCV array.
        self.br = CvBridge()

        # Marker size is the physical black-square side length in metres. PnP
        # translation scales directly with this value.
        self.pose_estimator = ArucoPoseEstimator(marker_size=0.20)

        # This helper composes map->drone, drone->camera, and camera->marker.
        self.global_transformer = ArucoGlobalTransformer()

        # Cache the most recent observation of each marker ID. The entire cache
        # is republished whenever a marker is updated. Expiration/deletion is a
        # separate future improvement; last_seen preserves the real image time.
        self.markers = {}

        # Retain the most recently received fixed camera calibration.
        self.info_sub = rospy.Subscriber(
            '/depthai_node/camera/camera_info',
            CameraInfo,
            self.info_callback,
            queue_size=1,
        )

        # Image publication must not depend on localisation. Retain the latest
        # pose separately and use it only while it is close enough to the image
        # timestamp for a defensible map-frame marker transform.
        self.latest_pose = None
        self.max_pose_age = rospy.get_param('~max_pose_age', 0.20)
        self.frame_sub = rospy.Subscriber(
            self.frame_sub_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2**24,
        )
        self.pose_sub = rospy.Subscriber(
            self.pose_sub_topic,
            PoseStamped,
            self.pose_callback,
            queue_size=1,
        )

        rospy.loginfo(
            "Processing every camera image; map transforms require a pose "
            "within %.3f s",
            self.max_pose_age,
        )

    def store_marker(self, marker_ID, T, rvec, tvec, image_stamp):
        """Cache one successful global observation and republish the cache.

        ``T`` is T_map_marker, so its final column is the marker origin written
        in map coordinates. rvec/tvec are retained for future diagnostics even
        and remain useful for image-overlay diagnostics.
        """

        marker_position = T[:3, 3]
        marker_orientation = R.from_matrix(T[:3, :3]).as_quat()

        marker_ID = int(marker_ID)

        if marker_ID not in self.markers:

            print(
                f"Discovered Aruco {marker_ID}"
                f" | World X: {marker_position[0]:.2f}"
                f" | World Y: {marker_position[1]:.2f}"
                f" | World Z: {marker_position[2]:.2f}"
            )

        self.markers[marker_ID] = {
            "id": marker_ID,
            "position": marker_position,
            # scipy and ROS both order quaternion components x, y, z, w.
            "orientation": marker_orientation,
            "transform": T,
            "rvec": rvec,
            "tvec": tvec,
            # Record when the observation was made, not when processing ended.
            "last_seen": image_stamp,
        }

        self.publish_marker_array()

    def publish_marker_array(self):
        """Publish all cached marker positions for RViz and downstream nodes."""
        marker_array = MarkerArray()

        for marker_data in self.markers.values():

            marker = Marker()

            # The position has already been transformed into map coordinates.
            # Use the observation time stored with this specific marker; using
            # rospy.Time.now() here would make an old cached pose look fresh.
            marker.header.frame_id = "map"
            marker.header.stamp = marker_data["last_seen"]

            # Namespace and integer ID form RViz's persistent marker identity.
            marker.ns = "aruco_markers"
            marker.id = marker_data["id"]

            # ADD creates the cube or replaces the prior cube with this ID.
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # Publish rotation from T_map_marker rather than concealing PnP or
            # camera-mount errors behind an identity quaternion.
            marker.pose.position.x = marker_data["position"][0]
            marker.pose.position.y = marker_data["position"][1]
            marker.pose.position.z = marker_data["position"][2]

            marker.pose.orientation.x = marker_data["orientation"][0]
            marker.pose.orientation.y = marker_data["orientation"][1]
            marker.pose.orientation.z = marker_data["orientation"][2]
            marker.pose.orientation.w = marker_data["orientation"][3]

            # Draw the same physical square size used by PnP.
            marker.scale.x = self.pose_estimator.marker_size
            marker.scale.y = self.pose_estimator.marker_size
            marker.scale.z = 0.01

            # Opaque red distinguishes detected ArUcos from map visualization.
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


    def pose_callback(self, pose_msg):
        """Retain the newest vehicle pose without gating image processing."""
        self.latest_pose = pose_msg

    def image_callback(self, image_msg):
        """Detect and republish every image, with or without localisation."""
        try:
            frame = self.br.compressed_imgmsg_to_cv2(image_msg)
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        pose_msg = self.latest_pose
        if pose_msg is not None:
            pose_age = abs(
                (image_msg.header.stamp - pose_msg.header.stamp).to_sec()
            )
            if pose_age > self.max_pose_age:
                pose_msg = None

        if pose_msg is None:
            rospy.logwarn_throttle(
                5.0,
                "No recent vehicle pose: publishing ArUco imagery without "
                "updating map-frame marker locations",
            )

        aruco = self.find_aruco(
            frame,
            pose_msg,
            image_msg.header.stamp,
        )
        self.publish_to_ros(aruco, image_msg.header)

        # cv2.imshow('aruco', aruco)
        # cv2.waitKey(1)

    def info_callback(self, msg):
        """Replace the PnP estimator's calibration with the latest CameraInfo."""
        self.pose_estimator.set_camera_info(msg)

    def find_aruco(self, frame, pose_msg, image_stamp):
        """Detect all markers and estimate each one's map-frame transform.

        Detection supplies ordered 2D corner pixels. ArucoPoseEstimator combines
        those pixels with marker geometry, K, and D to produce T_camera_marker.
        ArucoGlobalTransformer then combines it with a recent MAVROS pose and
        fixed camera mount to produce T_map_marker. If pose_msg is unavailable,
        detection and PnP overlays continue but the map marker is not updated.
        """
        detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        corners, ids, _ = detector.detectMarkers(frame)


        if len(corners) > 0:
            ids = ids.flatten()

            for (marker_corner, marker_ID) in zip(corners, ids):
                pts = marker_corner.reshape((4, 2))
                (top_left, top_right, bottom_right, bottom_left) = pts

                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                # Detection can begin before CameraInfo arrives, but PnP cannot.
                if self.pose_estimator.got_camera_info:
                    success, rvec, tvec = self.pose_estimator.estimate_pose(marker_corner)

                    if success:

                        T = None
                        if pose_msg is not None:
                            T = self.global_transformer.camera_to_world(
                                rvec,
                                tvec,
                                pose_msg,
                            )

                        if T is not None:

                            self.store_marker(
                                marker_ID,
                                T,
                                rvec,
                                tvec,
                                image_stamp,
                            )

                        self.pose_estimator.draw_pose_overlay(
                            frame,
                            rvec,
                            tvec,
                            top_left
                        )

                cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
                cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)

                # rospy.loginfo("Aruco detected, ID: {}".format(marker_ID))

                self.pose_estimator.draw_text_180(
                    frame,
                    str(marker_ID),
                    (top_left[0], top_right[1] - 15),
                    cv2.FONT_HERSHEY_COMPLEX,
                    0.5,
                    (0, 255, 0),
                    2,
                )

        return frame

    def publish_to_ros(self, frame, source_header):
        """JPEG-encode the overlay while preserving its source image header.

        Copying the header retains both the optical frame ID and the timestamp
        used to select pose_msg. A new publication-time stamp would break the
        traceability between this overlay and its original observation.
        """
        success, encoded = cv2.imencode('.jpg', frame)
        if not success:
            rospy.logwarn("Failed to encode processed ArUco image")
            return

        msg_out = CompressedImage()
        msg_out.header = source_header
        msg_out.format = "jpeg"
        msg_out.data = encoded.tobytes()

        self.aruco_pub.publish(msg_out)


def main():
    """Start the detector node and hand callback execution to rospy."""
    rospy.init_node('EGH450_vision', anonymous=True)
    rospy.loginfo("Processing images...")

    aruco_detect = ArucoDetector()

    rospy.spin()
