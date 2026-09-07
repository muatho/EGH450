#!/usr/bin/env python3

import os
import argparse
import cv2
import rosbag
import numpy as np

def main():

    parser = argparse.ArgumentParser(description="Extract video from a ROS bag.")
    parser.add_argument("bag_file")
    parser.add_argument("output_dir")
    parser.add_argument("image_topic")

    args = parser.parse_args()

    print(f"Extract video from {args.bag_file} on topic {args.image_topic} into {args.output_dir}")

    bag = rosbag.Bag(args.bag_file, "r")

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = None

    frame_count = 0
    sample_rate = 1.0   # keep every frame (no dropping)
    input_fps = 20.0   # figured out from number of frames / recording length
    
    start_time = 90
    end_time = 150
    bag_start = None

    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):

        frame_count += 1
        
        if bag_start is None:
            bag_start = t.to_sec()
            
        elapsed = t.to_sec() - bag_start
        
        if elapsed < start_time:
            continue
        
        if elapsed > end_time:
            break

        # drop frames
        if frame_count % sample_rate != 0:
            continue

        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if out is None:
            h, w = cv_img.shape[:2]
            out = cv2.VideoWriter(
                os.path.join(args.output_dir, "output_1.mp4"),
                fourcc,
                input_fps / sample_rate,   # 👈 THIS is the key fix
                (w, h)
            )

        out.write(cv_img)

    # 👇 IMPORTANT: release AFTER loop
    if out:
        out.release()

    bag.close()

if __name__ == '__main__':
    main()
