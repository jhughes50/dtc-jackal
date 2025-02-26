#!/usr/bin/python3
"""
    Jason Hughes
    September 2024

    A node to mask a person from an image

    DTC PRONTO 2024
"""
# LAST EDIT -- Daudi November 21 2024 for Infrared Testing
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

from skimage.transform import rotate, resize

from ultralytics import YOLO
from yolov8_ros.utils import boxes_2_cs

class PersonMasker:

    def __init__(self):

        self.detecting_ = False
        self.healthy_ = False
        
        #path = rospy.get_param("/masker_node/model_path")
        path = "/home/dtc/Docker/masking/yolov8-ros/models/yolov8l.pt"

        self.model_ = YOLO(path, verbose=False)

        self.crop_ = rospy.get_param("/masker_node/crop")
        
        self.bridge_ = CvBridge()
        rospy.Subscriber("/image", Image, self.imageCallback)
    
        self.mask_pub_ = rospy.Publisher("/image/masked", Image, queue_size=2)
        rospy.Timer(rospy.Duration(2), self.statusReport)
        
        print("[YOLOV8-ROS] Node initialized")


    def imageCallback(self, msg : Image) -> None:
        self.healthy_ = True
        cv_image = self.bridge_.imgmsg_to_cv2(msg, "mono16") # initially: bgr8 
        #np_arr = np.frombuffer(msg.data, np.uint8)
        #cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        H, W = cv_image.shape[:2]
        cv_image = cv_image.astype(np.uint8)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)
        output = self.model_(cv_image, verbose=False)[0]

        boxes = output.boxes
        masks = output.masks

        valid = boxes.cls == 0
        if valid.any():
            self.detecting_ = True
            boxes = boxes[valid].data.cpu()
            masks = masks[valid].data.cpu()
            keep = 1
            centers, scales = boxes_2_cs(boxes)
            idx = np.argsort(scales)[::-1].copy()
            boxes = boxes[idx][:keep].squeeze()
            masks = masks[idx][:keep][0]
            mask = resize(masks.numpy(), (H, W))

            m_arr = (mask * 65535).astype(np.uint16) # initially (mask * 255) and uint8
                       
            m_img = cv2.bitwise_and(cv_image, cv_image, mask=m_arr)

            if self.crop_:
                boxes = boxes.int().tolist()
                #print(boxes)
                m_img = m_img[boxes[1]:boxes[3], boxes[0]:boxes[2],:]


            m_msg = self.bridge_.cv2_to_imgmsg(m_img, 'mono16') # initially bgr8
            m_msg.header = msg.header
           
            self.mask_pub_.publish(m_msg)
        else:
            self.detecting_ = False

    def statusReport(self, event=None):
        print("[YOLOV8-ROS] Recieving Images: %s" %self.healthy_)
        print("[YOLOV8-ROS] Currently Processing: %s" %self.detecting_)



if __name__ == "__main__":
    rospy.init_node("masker")
    
    PersonMasker()

    rospy.spin()

