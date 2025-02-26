"""
    Jason Hughes
    September 2024

    ROS wrapper for MTTS-CAN

    DTC PRONTO 2024
"""
import sys
import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32, UInt8
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from typing import Tuple

import tensorflow as tf
import numpy as np
import cv2
import scipy.io
import os
import sys
import argparse
sys.path.append('../')
from mtts_can_ros.model import Attention_mask, MTTS_CAN, TSM
import h5py
import matplotlib.pyplot as plt
from scipy.signal import butter
from skimage.util import img_as_float
from mtts_can_ros.inference_preprocess import detrend

from tensorflow.keras.models import load_model


class HeartNetRos:

    def __init__(self) -> None:

        self.bridge_ = CvBridge()
        self.frame_rate_ = rospy.get_param("/mtts_hr/mtts-can/frame_rate")
        self.duration_ = rospy.get_param("/mtts_hr/mtts-can/duration") # time in seconds to take a reading
        self.dim_ = rospy.get_param("/mtts_hr/mtts-can/dim")
        self.batch_size_ = rospy.get_param("/mtts_hr/mtts-can/batch_size")
        self.distance_ = rospy.get_param("/mtts_hr/mtts-can/distance")
        path = rospy.get_param("mtts_hr/path")                            

        gpus = tf.config.experimental.list_physical_devices("GPU")

        if gpus:
            for gpu in gpus:
                tf.config.experimental.set_memory_growth(gpu, True)

        self.trigger_ = None
        self.processing_ = False
        self.complete_ = None
        
        self.model_ = MTTS_CAN(5, 32, 64, (self.dim_, self.dim_, 3))
        self.model_.load_weights(path)

        self.buffer_ = np.zeros((self.frame_rate_*self.duration_, self.dim_, self.dim_, 3))
        self.buffer_count_ = 0

        rospy.Subscriber("/image", Image, self.imageCallback)
        rospy.Subscriber("/jackal_teleop/trigger", UInt8, self.triggerCallback)

        self.hr_pub_ = rospy.Publisher("/heart_rate/model", PointStamped, queue_size=2)
        self.rr_pub_ = rospy.Publisher("/respiration_rate/model", PointStamped, queue_size=2)

        rospy.Timer(rospy.Duration(2.0), self.statusCallback)
        rospy.loginfo("[MTTS-ROS] Heart Rate Net initialized")

    def statusCallback(self, call) -> None:
        print("[MTTS-ROS] Trigger : ", self.trigger_, " Processing: ", self.processing_, " Complete ", self.complete_, "  ", end='\r')

    def preprocessFrame(self, frame : np.ndarray, height : int, width : int) -> np.ndarray:
        rframe = cv2.resize(img_as_float(frame[:, width//2 - (height//2+1):height//2+width//2,:]), (self.dim_, self.dim_), interpolation=cv2.INTER_AREA)
        rframe = cv2.rotate(rframe, cv2.ROTATE_90_CLOCKWISE)
        rframe = cv2.cvtColor(rframe.astype('float32'), cv2.COLOR_BGR2RGB)
        rframe[rframe > 1] = 1
        rframe[rframe < (1/255)] = 1/255

        return rframe

    def preprocessSequence(self, seq : np.ndarray) -> np.ndarray:
        d_seq = np.zeros((self.frame_rate_*self.duration_, self.dim_, self.dim_, 3), dtype=np.float32)
        for j in range(self.frame_rate_*self.duration_ - 1):
            d_seq[j, :, :, :] = (seq[j+1, :, :, :] - seq[j, :, :, :]) / (seq[j+1, :, :, :] + seq[j, :, :, :])

        d_seq = d_seq / np.std(d_seq)
        # Normalize raw frames
        seq = seq - np.mean(seq)
        seq = seq / np.std(seq)
        d_seq = np.concatenate((d_seq, seq), axis=3)

        return d_seq

    def imageCallback(self, msg : Image) -> None:
        if self.trigger_ != 0 and self.processing_:
            h, w = msg.height, msg.width
            img = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')
            processed_frame = self.preprocessFrame(img, h, w)

            self.buffer_[self.buffer_count_,:,:,:] = processed_frame
            
            if self.buffer_count_ == (self.frame_rate_ * self.duration_ - 1):
                print("[MTTS-ROS] Processing sequence")
                d_seq = self.preprocessSequence(self.buffer_)
                pulse, resp = self.process(d_seq)
                
                peaks = scipy.signal.find_peaks(pulse, distance = self.distance_)
                time = 60 / self.duration_
                hr = len(peaks[0])*time

                rpeaks = scipy.signal.find_peaks(resp, distance = 100)
                rr = len(rpeaks[0])*time

                hrmsg = PointStamped()
                hrmsg.header.stamp = rospy.Time.now()
                hrmsg.header.frame_id = "mtts-can"
                hrmsg.point.x = hr
                self.hr_pub_.publish(hrmsg)

                rrmsg = PointStamped()
                rrmsg.header.stamp = rospy.Time.now()
                rrmsg.header.frame_id = "mtts-can"
                rrmsg.point.x = rr
                self.rr_pub_.publish(rrmsg)

                #rospy.loginfo("[MTTS-ROS] Heart Rate Complete")
                self.complete_ = True
                self.processing_ = False
                self.buffer_count_ = 0
            else:
                self.complete_ = False
                self.processing_ = True
                self.buffer_count_ += 1


    def process(self, d_seq : np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        yptest = self.model_.predict((d_seq[:, :, :, :3], d_seq[:, :, :, -3:]), batch_size=self.batch_size_)

        pulse_pred = yptest[0]
        pulse_pred = detrend(np.cumsum(pulse_pred), 100)
        [b_pulse, a_pulse] = butter(1, [0.75 / self.frame_rate_ * 2, 2.5 / self.frame_rate_ * 2], btype='bandpass')
        pulse_pred = scipy.signal.filtfilt(b_pulse, a_pulse, np.double(pulse_pred))

        resp_pred = yptest[1]
        resp_pred = detrend(np.cumsum(resp_pred), 100)
        [b_resp, a_resp] = butter(1, [0.08 / self.frame_rate_ * 2, 0.5 / self.frame_rate_ * 2], btype='bandpass')
        resp_pred = scipy.signal.filtfilt(b_resp, a_resp, np.double(resp_pred))
       
        return pulse_pred, resp_pred

    def triggerCallback(self, msg : UInt8) -> None:
        self.trigger_ = msg.data
        if self.trigger_ > 0:
            self.processing_ = True
        else:
            self.processing_ = False

if __name__ == "__main__":
    rospy.init_node("mtts_can_node")
    HeartNetRos()

    rospy.spin()

