#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
Example use:  python3 bags2images.py '/home/user/2023-07-31-14-30-34.bag' ~/Desktop/testOutFrames /depthai_node/image/raw
"""

import os
import argparse
import numpy as np

import cv2

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")

    args = parser.parse_args()

    print ("Extract images from %s on topic %s into %s" % (args.bag_file,
                                                          args.image_topic, args.output_dir))

    bag = rosbag.Bag(args.bag_file, "r")
    bridge = CvBridge()
    count = 0
    start_time = 90
    end_time = 91
    bag_start = None
    
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):

        if bag_start is None:
            bag_start = t.to_sec()

        elapsed = t.to_sec() - bag_start

        if elapsed < start_time:
            continue

        if elapsed > end_time:
            break


        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        cv2.imwrite(os.path.join(args.output_dir, "frame%06i.png" % count), cv_img)
        print ("Wrote image %i" % count)

        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()
