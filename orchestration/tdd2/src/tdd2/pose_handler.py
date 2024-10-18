"""
    Jason Hughes
    August 2024
    
    Parent node to handle the pose and 
    orientation callbacks for localization

    DTC PRONTO 2024
"""

import rospy
import copy
import numpy as np
import math

from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as R


class PoseHandler:

    def __init__(self) -> None:

        self.hfov_ = rospy.get_param("sync/cam0/hfov")
        self.width_ = rospy.get_param("sync/cam0/resolution")[0]
        self.resolution_ = 0.0

        rospy.Subscriber("/mavros/global_position/local", Odometry, self.odomCallback)
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, self.gpsCallback)
        rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.headingCallback)
        rospy.Subscriber("/mavros/global_position/rel_alt", Float64, self.altitudeCallback)

        self.current_pose_ = Pose()
        self.current_heading_ = 0.0
        self.current_gps_ = NavSatFix()

        self.altitude_ = 0.0

        self.rotation_matrix_ = np.eye(3)
        self.translation_ = np.ones(3)

        self.extrinsics_ = np.ones((3,4))
        self.extrinsics_homo_ = np.eye(4)

    def odomCallback(self, msg : Odometry) -> None:
        
        self.current_pose_ = msg.pose.pose
        #self.translation_ = np.array([msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.position.z])
        self.translation_ = np.array([0,0,msg.pose.pose.position.z])
        self.updateExtrinsics()

    def gpsCallback(self, msg : NavSatFix) -> None:
        self.current_gps_ = msg

    def headingCallback(self, msg : Float64) -> None: 
        self.current_heading_ = msg.data

    def altitudeCallback(self, msg : Float64) -> None:
        self.altitude_ = msg.data
        img_width_world = 2.0 * (self.altitude_ * math.tan(self.hfov_/2))
        self.resolution_ = img_width_world / self.width_

    def updateExtrinsics(self) -> None:
        r = R.from_euler('y', [self.current_heading_], degrees=True)
        self.rotation_matrix_ = r.as_matrix()

        ex = np.ones((3,4))
        ex[:3,:3] = np.eye(3)#self.rotation_matrix_
        ex[:,-1] = self.translation_.T
        
        bottom_row = np.array([[0,0,0,1]])

        self.extrinsics_ = copy.copy(ex)
        self.extrinsics_homo_ = copy.copy(np.vstack((ex, bottom_row)))
