#!/usr/bin/env python3

import json
from pathlib import Path

import cv2

import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose

from cv_bridge import CvBridge
import depthai as dai

import numpy as np


class DepthaiCamera():
    """Publish OAK RGB images and matching, device-read camera calibration.

    The class owns the DepthAI pipeline, the ROS publishers, and one CameraInfo
    template populated from calibration stored in the OAK. Calibration values
    stay constant, but the header timestamp is updated for every image.
    """

    def __init__(self):
        """Create ROS publishers and describe the pipeline before opening it."""
        # These dimensions are used consistently for the DepthAI output and
        # CameraInfo. Explicit names avoid width/height ambiguity.
        self.width = 640
        self.height = 360
        self.fps = 20.0

        # YOLO is normally enabled, but can be disabled from the launch file
        # to isolate camera and ArUco behaviour during diagnosis.
        self.enable_yolo = rospy.get_param("~enable_yolo", True)
        self.yolo_topic = "/yolo/detections"

        # This YOLOv5 model is temporary while YOLOv8 is being trained.
        package_dir = Path(__file__).resolve().parent
        model_dir = package_dir / "models" / "best_openvino_2022.1_6shave"
        self.yolo_config_path = Path(rospy.get_param(
            "~yolo_config_path", str(model_dir / "best.json")
        )).expanduser()
        self.yolo_blob_path = Path(rospy.get_param(
            "~yolo_blob_path",
            str(model_dir / "best_openvino_2022.1_6shave.blob"),
        )).expanduser()

        # Images and CameraInfo describe the same optical camera, so all use
        # the same ROS frame name.
        self.image_frame = "oak_rgb_camera_optical_frame"

        # CAM_A is the RGB sensor on this OAK, confirmed by the stored 4K
        # calibration. K and D must both come from this same sensor socket.
        self.rgb_socket = dai.CameraBoardSocket.CAM_A

        self.pub_topic = '/depthai_node/image/compressed'
        self.pub_topic_raw = '/depthai_node/image/raw'
        self.pub_topic_cam_info = '/depthai_node/camera/camera_info'

        # queue_size=1 favours the newest frame if a subscriber falls behind.
        self.pub_image = rospy.Publisher(self.pub_topic, CompressedImage, queue_size=1)
        self.pub_image_raw = rospy.Publisher(self.pub_topic_raw, Image, queue_size=1)
        self.pub_cam_info = rospy.Publisher(self.pub_topic_cam_info, CameraInfo, queue_size=1)
        self.pub_yolo = None
        if self.enable_yolo:
            self.pub_yolo = rospy.Publisher(
                self.yolo_topic, Detection2DArray, queue_size=1
            )

        # CvBridge converts between OpenCV arrays and ROS Image messages.
        self.bridge = CvBridge()

        # A pipeline describes what the OAK will do when a device is opened;
        # creating this object does not connect to the physical camera yet.
        self.pipeline = dai.Pipeline()

        # Populated from EEPROM only after run() opens the physical device.
        self.camera_info = None

        self.yolo_width = None
        self.yolo_height = None
        if self.enable_yolo:
            self.read_yolo_config()

        self.configure_pipeline()

        rospy.loginfo(
            "Configured %dx%d RGB images for %s",
            self.width,
            self.height,
            self.pub_topic,
        )

        rospy.on_shutdown(self.shutdown)

    def read_yolo_config(self):
        """Read the temporary YOLOv5 model's DepthAI decoder settings.

        These anchors are specific to this old model. A future YOLOv8 export
        needs its own compatible output and decoder configuration.
        """
        if not self.yolo_config_path.is_file():
            raise FileNotFoundError(
                "YOLO config does not exist: {}".format(self.yolo_config_path)
            )
        if not self.yolo_blob_path.is_file():
            raise FileNotFoundError(
                "YOLO blob does not exist: {}".format(self.yolo_blob_path)
            )

        with self.yolo_config_path.open("r") as config_file:
            config = json.load(config_file)

        nn_config = config["nn_config"]
        metadata = nn_config["NN_specific_metadata"]
        dimensions = nn_config["input_size"].lower().split("x")
        if len(dimensions) != 2:
            raise ValueError("Invalid YOLO input_size")

        self.yolo_width = int(dimensions[0])
        self.yolo_height = int(dimensions[1])
        self.yolo_classes = int(metadata["classes"])
        self.yolo_coordinates = int(metadata["coordinates"])
        self.yolo_anchors = metadata["anchors"]
        self.yolo_anchor_masks = metadata["anchor_masks"]
        self.yolo_iou_threshold = float(metadata["iou_threshold"])
        self.yolo_confidence_threshold = float(
            metadata["confidence_threshold"]
        )
        self.yolo_labels = config["mappings"]["labels"]
        if len(self.yolo_labels) != self.yolo_classes:
            raise ValueError("YOLO label count does not match class count")

    def configure_pipeline(self):
        """Build the on-device RGB-to-host data path.

        The sensor operates in 1080p mode. Its preview is scaled to 640 x 360,
        preserving 16:9. XLinkOut transfers frames to the host under the name
        ``video``; run() opens a queue with that same name.
        """
        cam_rgb = self.pipeline.createColorCamera()

        # Select the sensor operating mode.
        cam_rgb.setResolution(
            dai.ColorCameraProperties.SensorResolution.THE_1080_P
        )

        # Select the actual image dimensions leaving the device.
        cam_rgb.setPreviewSize(self.width, self.height)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(self.fps)

        xout_rgb = self.pipeline.createXLinkOut()
        xout_rgb.setStreamName("video")

        cam_rgb.preview.link(xout_rgb.input)

        if self.enable_yolo:
            self.configure_yolo_pipeline(cam_rgb)

    def configure_yolo_pipeline(self, cam_rgb):
        """Create the letterboxed 640x360 camera-to-YOLO branch."""
        manip = self.pipeline.createImageManip()
        manip.initialConfig.setResizeThumbnail(
            self.yolo_width, self.yolo_height
        )
        manip.setMaxOutputFrameSize(self.yolo_width * self.yolo_height * 3)
        cam_rgb.preview.link(manip.inputImage)

        network = self.pipeline.createYoloDetectionNetwork()
        network.setBlobPath(str(self.yolo_blob_path))
        network.setConfidenceThreshold(self.yolo_confidence_threshold)
        network.setNumClasses(self.yolo_classes)
        network.setCoordinateSize(self.yolo_coordinates)
        network.setAnchors(self.yolo_anchors)
        network.setAnchorMasks(self.yolo_anchor_masks)
        network.setIouThreshold(self.yolo_iou_threshold)
        network.input.setBlocking(False)
        network.setNumInferenceThreads(2)
        manip.out.link(network.input)

        xout_yolo = self.pipeline.createXLinkOut()
        xout_yolo.setStreamName("yolo_detections")
        xout_yolo.input.setBlocking(False)
        network.out.link(xout_yolo.input)

    def read_camera_info(self, device):
        """Read RGB EEPROM calibration and build a ROS CameraInfo message.

        getCameraIntrinsics() expresses K in this publisher's 640 x 360 pixel
        coordinates, so no manual division or hard-coded focal length is used.
        The first eight returned distortion values are the ROS rational model:
        k1, k2, p1, p2, k3, k4, k5, k6.
        """
        calibration = device.readCalibration()

        intrinsic_matrix = calibration.getCameraIntrinsics(
            self.rgb_socket,
            self.width,
            self.height,
        )

        distortion_coefficients = (
            calibration.getDistortionCoefficients(self.rgb_socket)
        )

        intrinsic_matrix = np.asarray(
            intrinsic_matrix,
            dtype=float,
        ).reshape(3, 3)

        distortion_coefficients = np.asarray(
            distortion_coefficients,
            dtype=float,
        ).reshape(-1)

        # CameraInfo is only a ROS container. The values inserted here came
        # from the DepthAI calibration API above.
        camera_info = CameraInfo()

        camera_info.width = self.width
        camera_info.height = self.height
        camera_info.distortion_model = "rational_polynomial"

        # ROS rational_polynomial uses:
        # k1, k2, p1, p2, k3, k4, k5, k6
        camera_info.D = distortion_coefficients[:8].tolist()
        camera_info.K = intrinsic_matrix.reshape(-1).tolist()

        # Identity R means this publisher is not rotating the RGB image into a
        # different rectified camera orientation.
        camera_info.R = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0,
        ]

        fx = intrinsic_matrix[0, 0]
        skew = intrinsic_matrix[0, 1]
        cx = intrinsic_matrix[0, 2]
        fy = intrinsic_matrix[1, 1]
        cy = intrinsic_matrix[1, 2]

        # Monocular 3 x 4 projection matrix: no stereo baseline translation.
        camera_info.P = [
            fx, skew, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]

        camera_info.header.frame_id = self.image_frame

        rospy.loginfo(
            "Read RGB calibration from OAK for %dx%d",
            self.width,
            self.height,
        )

        rospy.loginfo("K:\n%s", intrinsic_matrix)
        rospy.loginfo(
            "D rational coefficients:\n%s",
            distortion_coefficients[:8],
        )

        return camera_info

    def run(self):
        """Open the OAK, read calibration once, then publish incoming frames."""
        # EEPROM cannot be read until the physical device has been opened.
        with dai.Device(self.pipeline) as device:
            self.camera_info = self.read_camera_info(device)

            # Retain the newest host-bound frame rather than accumulating old
            # frames when the host temporarily cannot keep up.
            video = device.getOutputQueue(
                name="video", maxSize=1, blocking=False)

            yolo_queue = None
            if self.enable_yolo:
                yolo_queue = device.getOutputQueue(
                    name="yolo_detections", maxSize=1, blocking=False
                )
            stamps_by_sequence = {}

            while not rospy.is_shutdown():
                packet = video.get()
                frame = packet.getCvFrame()

                # One timestamp is shared by raw Image, CompressedImage, and
                # CameraInfo. This replaces the old independent 10 Hz timer.
                stamp = rospy.Time.now()
                self.publish_to_ros(frame, stamp)

                if self.enable_yolo:
                    sequence = packet.getSequenceNum()
                    stamps_by_sequence[sequence] = stamp
                    stamps_by_sequence = {
                        key: value
                        for key, value in stamps_by_sequence.items()
                        if key >= sequence - 100
                    }

                    detections = yolo_queue.tryGet()
                    while detections is not None:
                        detection_stamp = stamps_by_sequence.get(
                            detections.getSequenceNum(), stamp
                        )
                        self.publish_yolo_detections(
                            detections, detection_stamp
                        )
                        detections = yolo_queue.tryGet()

    def publish_yolo_detections(self, packet, stamp):
        """Publish boxes in the original 640x360 image coordinates."""
        output = Detection2DArray()
        output.header.stamp = stamp
        output.header.frame_id = self.image_frame

        # Undo the scale and padding from setResizeThumbnail().
        scale = min(
            self.yolo_width / float(self.width),
            self.yolo_height / float(self.height),
        )
        pad_x = (self.yolo_width - self.width * scale) / 2.0
        pad_y = (self.yolo_height - self.height * scale) / 2.0

        for result in packet.detections:
            x_min = np.clip(
                (result.xmin * self.yolo_width - pad_x) / scale,
                0, self.width,
            )
            y_min = np.clip(
                (result.ymin * self.yolo_height - pad_y) / scale,
                0, self.height,
            )
            x_max = np.clip(
                (result.xmax * self.yolo_width - pad_x) / scale,
                0, self.width,
            )
            y_max = np.clip(
                (result.ymax * self.yolo_height - pad_y) / scale,
                0, self.height,
            )
            if x_max <= x_min or y_max <= y_min:
                continue

            detection = Detection2D()
            detection.header = output.header
            detection.bbox.center.x = float((x_min + x_max) / 2.0)
            detection.bbox.center.y = float((y_min + y_max) / 2.0)
            detection.bbox.center.theta = 0.0
            detection.bbox.size_x = float(x_max - x_min)
            detection.bbox.size_y = float(y_max - y_min)

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = int(result.label)
            hypothesis.score = float(result.confidence)
            hypothesis.pose.pose.orientation.w = 1.0
            detection.results.append(hypothesis)
            output.detections.append(detection)

        self.pub_yolo.publish(output)

    def publish_to_ros(self, frame, stamp):
        """Publish a frame in raw and JPEG form with matching CameraInfo."""
        # JPEG saves transport bandwidth; the raw message below retains the
        # original pixels for local computer-vision consumers.
        success, encoded = cv2.imencode(".jpg", frame)

        if not success:
            rospy.logwarn("Failed to JPEG-encode camera frame")
            return

        compressed_msg = CompressedImage()
        compressed_msg.header.stamp = stamp
        compressed_msg.format = "jpeg"
        compressed_msg.header.frame_id = self.image_frame
        compressed_msg.data = encoded.tobytes()

        # Convert the same OpenCV array into an uncompressed ROS Image.
        raw_msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding="bgr8",
        )
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = self.image_frame

        # K, D, R, and P remain constant. Updating this header associates that
        # calibration explicitly with the current frame.
        self.camera_info.header.stamp = stamp
        self.camera_info.header.frame_id = self.image_frame

        self.pub_image.publish(compressed_msg)
        self.pub_image_raw.publish(raw_msg)
        self.pub_cam_info.publish(self.camera_info)


    def shutdown(self):
        """Close any OpenCV windows when ROS stops the node."""
        cv2.destroyAllWindows()


def main():
    """Initialise the ROS node and run one DepthaiCamera instance."""
    rospy.init_node("depthai_node")

    camera = DepthaiCamera()
    camera.run()


if __name__ == "__main__":
    main()
