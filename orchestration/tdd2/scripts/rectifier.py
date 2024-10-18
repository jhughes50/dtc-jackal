"""
    Jason Hughes
    September 2024

    Quick Node to rectify the images

    DTC PRONTO 2024

"""

import rospy
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class ImageRectifier:

    def __init__(self):
        self.K_ = np.eye(3)
        
        self.model_ = rospy.get_param("/sync/cam0/distortion_model")
        self.distortion_ = np.array(rospy.get_param("/sync/cam0/distortion_coeffs"))

        self.bridge_ = CvBridge()
        self.initIntrinsics()

        rospy.Subscriber("/image_in", Image, self.imageCallback)

        self.rect_pub_ = rospy.Publisher("/image_out", Image, queue_size=1)

    def initIntrinsics(self):
        k_params = rospy.get_param("/sync/cam0/intrinsics")

        self.K_[0, 0] = k_params[0]
        self.K_[1, 1] = k_params[1]
        self.K_[0, 2] = k_params[2]
        self.K_[1, 2] - k_params[3]

    def imageCallback(self, msg : Image) -> None:
        
        try:
            img = self.bridge_.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print("[RECTIFIER] CV Bridge Error: %s" %e)

        if self.model_ == "equidistant":
            undistorted = cv2.fisheye.undistortImage(img, self.K_, self.distortion_, None, self.K_)
        elif self.model_ == "radtan":
            undistorted = cv2.undistort(img, self.K_, self.distortion_, None, self.K_)

        pub_msg = self.bridge_.cv2_to_imgmsg(undistorted, "bgr8")
        pub_msg.header = msg.header
        self.rect_pub_.publish(pub_msg)

if __name__ == "__main__":
    
    try:
        rospy.init_node("image_rectifier_node")

        ImageRectifier()

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
