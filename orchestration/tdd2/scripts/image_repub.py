#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageRepublisher:
    def __init__(self):
        # Initialize node
        rospy.init_node('image_republisher', anonymous=True)

        # Create a CvBridge object to convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

        # Subscriber for the compressed image topic
        self.image_sub = rospy.Subscriber("/camera/image_color/compressed", CompressedImage, self.image_callback)

        # Subscriber for the camera info topic
        self.camera_info_sub = rospy.Subscriber("/camera/camera_info", CameraInfo, self.camera_info_callback)

        # Publisher for the raw image topic
        self.image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)

        # Publisher for the camera info topic with the same header
        self.camera_info_pub = rospy.Publisher("/camera/raw_camera_info", CameraInfo, queue_size=1)

        # Variable to store the last received CameraInfo
        self.camera_info = None

    def image_callback(self, msg):
        # Convert the compressed image to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Convert the OpenCV image to a ROS Image message (raw format)
        raw_image_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")

        # Set the header of the raw image message to match the compressed image
        raw_image_msg.header = msg.header

        # Publish the raw image
        self.image_pub.publish(raw_image_msg)

        # Publish the CameraInfo message if we have received it
        # Set the header of the CameraInfo to match the image header
        self.camera_info.header = msg.header
        self.camera_info_pub.publish(self.camera_info)

    def camera_info_callback(self, msg):
        # Store the latest camera info
        self.camera_info = msg

if __name__ == '__main__':
    try:
        # Create an instance of the republisher class
        republisher = ImageRepublisher()

        # Keep the node running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
