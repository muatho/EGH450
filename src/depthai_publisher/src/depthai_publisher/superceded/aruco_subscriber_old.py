#!/usr/bin/env python3

import cv2

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import numpy as np

# Added for PnP pose estimation
from depthai_publisher.aruco_pose_estimator_PnP import ArucoPoseEstimator
from sensor_msgs.msg import CameraInfo



class ArucoDetector():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    aruco_params = cv2.aruco.DetectorParameters()
    
    frame_sub_topic = '/depthai_node/image/compressed'

    def __init__(self):
        self.aruco_pub = rospy.Publisher(
            '/processed_aruco/image/compressed', CompressedImage, queue_size=10)

        self.br = CvBridge()

        # Added for PnP pose estimation
        self.pose_estimator = ArucoPoseEstimator(marker_size=0.200)
        self.info_sub = rospy.Subscriber('/depthai_node/camera/camera_info', CameraInfo, self.info_callback)


        if not rospy.is_shutdown():
            self.frame_sub = rospy.Subscriber(
                self.frame_sub_topic, CompressedImage, self.img_callback)

    def img_callback(self, msg_in):
        try:
            frame = self.br.compressed_imgmsg_to_cv2(msg_in)
        except CvBridgeError as e:
            rospy.logerr(e)

        aruco = self.find_aruco(frame)
        self.publish_to_ros(aruco)

        # cv2.imshow('aruco', aruco)
        # cv2.waitKey(1)

    # Added for PnP pose estimation
    def info_callback(self, msg):
        self.pose_estimator.set_camera_info(msg)

    def find_aruco(self, frame):  
    
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

                # Added for PnP pose estimation
                if self.pose_estimator.got_camera_info:
                    success, rvec, tvec = self.pose_estimator.estimate_pose(marker_corner)
                    # print("pose success:", success)

                    if success:
                        X, Y, Z = tvec.flatten()
                        print("Aruco detected, ID:", marker_ID, f"| X: {X:.2f} m | Y: {Y:.2f} m | Z: {Z:.2f} m")
                        
                        self.pose_estimator.draw_pose_overlay(frame, rvec, tvec, top_left)
                        # print("pose success:", success)

                cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
                cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)

                # rospy.loginfo("Aruco detected, ID: {}".format(marker_ID))

                cv2.putText(frame, str(
                    marker_ID), (top_left[0], top_right[1] - 15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 2)

        return frame

    def publish_to_ros(self, frame):
        msg_out = CompressedImage()
        msg_out.header.stamp = rospy.Time.now()
        msg_out.format = "jpeg"
        msg_out.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()

        self.aruco_pub.publish(msg_out)


def main():
    rospy.init_node('EGB349_vision', anonymous=True)
    rospy.loginfo("Processing images...")

    aruco_detect = ArucoDetector()

    rospy.spin()
