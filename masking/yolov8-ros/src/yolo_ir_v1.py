#!/usr/bin/env python3

import rospy
import torch
import cv2
import numpy as np
from vision_msgs.msg import Detection2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from transformers import YolosConfig, YolosForObjectDetection
from yolov8_ros.IR_utils import custom_post_process

config = YolosConfig()
config.num_channels = 1
model = YolosForObjectDetection(config=config)
model.load_state_dict(torch.load('/home/dtc/ws/src/yolov8-ros/models/yolos_model_V2_Final300.pth', weights_only=True, map_location='cpu'))
model.eval()

model = model.to("cuda")

rospy.init_node('yolo_ir_detector', anonymous=True)
bridge = CvBridge()

pub_image = rospy.Publisher("yolo_detected_image", Image, queue_size=1)
#pub_bbox = rospy.Publisher("yolo_bbox_info", Float32MultiArray, queue_size=1)

def callback(image_msg):
    img = bridge.imgmsg_to_cv2(image_msg, "mono16")
    img_cp = img.copy()
    height, width = img.shape
    img = torch.from_numpy(img)
    img = torch.unsqueeze(img,0)
    img = torch.unsqueeze(img,0)
    img = img.float()
    predictions = model(img.to("cuda"))
    bbox_data = []
    result = custom_post_process(predictions, img_height = height, img_width = width)
    detections = result[0]
    #print("detections:", detections)
    num_detections = len(detections['scores'])
    if num_detections > 0:
        print("num detections:", num_detections)
    else:
        print("NADA!")
    img = cv2.normalize(img_cp, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    for i in range(num_detections):
        score = detections['scores'][i].item()
        label = detections['labels'][i].item()
        box = detections['boxes']
        box = box.tolist()
        if isinstance(box[0], list):
            box = box[0]

        x1, x2, y1, y2 = [int(round(float(j))) for j in box]
        
        #print(f"Class {label}, Confidence {round(score, 3)}, Box: {box}")
        
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        img_text = f"Class {label}: {round(score, 3)}"
        cv2.putText(img, img_text, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        x_center = round((x1 + x2) / 2, 2)
        y_center = round((y1 + y2) / 2, 2)

        bbox_data.extend([x1, y1, x2, y2, x_center, y_center])
    
    #bbox_msg = Detection2D(data=bbox_data)
    #pub_bbox.publish(bbox_msg)
    img = np.uint8(img)
    pub_image.publish(bridge.cv2_to_imgmsg(img, "bgr8"))
    #pub_image.publish(image_msg)
    


rospy.Subscriber("/camera/boson", Image, callback)
rospy.spin()
