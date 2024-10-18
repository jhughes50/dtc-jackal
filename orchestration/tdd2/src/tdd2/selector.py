
"""
    Jason Hughes
    August 2024

    DTC PRONTO 2024

"""

import math
import rospy
import numpy as np
import apriltag
import cv2

from typing import Tuple

from sensor_msgs.msg import Image, NavSatFix, CompressedImage

from tdd2.gps_conversion import LLtoUTM, UTMtoLL
from tdd2.yolo_localizer import YoloLocalization
from tdd2.pose_handler import PoseHandler
from tdd2.pose_frames import PoseFrames
from tdd2.msg import TDDetection


class SelectionHandler:

    def __init__(self):

        self.height_ = rospy.get_param("sync/cam0/resolution")[1]
        self.width_ = rospy.get_param("sync/cam0/resolution")[0]

        self.center_x_ = self.width_ // 2
        self.center_y_ = self.height_ // 2

        self.threshold_ = 100 #pixels
        
        self.yolo_ = rospy.get_param("yolo")
        self.intrinsics_ = rospy.get_param("/sync/cam0/intrinsics")
        
        options = apriltag.DetectorOptions(families='tag36h11')
        self.detector_ = apriltag.Detector(options)

        rospy.Subscriber("/image", CompressedImage, self.imageCallback)

    def imageCallback(self, msg : CompressedImage) -> None:
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        gs_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        result = self.detector_.detect(gs_img)

        for d in result:
            if d.center[0] < self.center_x_ + self.threshold_ and d.center[0] > self.center_x_ - self.threshold_ and d.center[1] > self.center_y_ + self.threshold_ and d.center[1] < self.center_y_ - self.threshold_:
                print("IN CENTER ID: ", d.tag_id)
