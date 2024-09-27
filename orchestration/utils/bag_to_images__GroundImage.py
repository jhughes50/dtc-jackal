#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2016 Massachusetts Institute of Technology

"""Extract images from a rosbag.
"""

import os
import argparse

import cv2
import numpy as np

import rosbag
from sensor_msgs.msg import Image, CompressedImage
from gone.msg import GroundImage
from cv_bridge import CvBridge

def decode_img_msg(msg: CompressedImage) -> np.ndarray:
    if True:  # better way? 
        np_arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
        print(img.shape)
    else:
        assert msg.encoding == "rgb8", "other values not supported"
        img = np.copy(
            np.ndarray(shape=(msg.height, msg.width, 3), dtype=np.uint8, buffer=msg.data)
        )
    return img

def main():
    """Extract a folder of images from a rosbag.
    """
    parser = argparse.ArgumentParser(description="Extract images from a ROS bag.")
    parser.add_argument("bag_file", help="Input ROS bag.")
    parser.add_argument("output_dir", help="Output directory.")
    parser.add_argument("image_topic", help="Image topic.")
    path = "/home/jason/data/data_colect_sept13/"

    args = parser.parse_args()

#    print("Extract images from %s on topic %s into %s" % (args.bag_file,
#                                                          args.image_topic, args.output_dir))
    bag = rosbag.Bag(args.bag_file)
    count = 0
    for topic, msg, t in bag.read_messages(topics=[args.image_topic]):
        np_img = decode_img_msg(msg.image)

        #cv_img = bridge.imgmsg_to_cv2(msg, "32FC1")
        t = "id_"+str(msg.casualty_id.data)+"_"+str(t.secs)
        print(t)
        cv2.imwrite(os.path.join(args.output_dir, "%s.png" %t), np_img)
        print ("Wrote image %i" % count)

        count += 1

    bag.close()

    return

if __name__ == '__main__':
    main()
