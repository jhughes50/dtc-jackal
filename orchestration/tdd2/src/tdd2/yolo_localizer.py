"""
    Jason Hughes
    August 2024
    Module to localize the detections 
    Based on the camera geometry

    DTC PRONTO 2024
"""

import math
import rospy
import numpy as np

from typing import Tuple

from std_msgs.msg import Float64
from geometry_msgs.msg import Pose

from tdd2.gps_conversion import LLtoUTM, UTMtoLL
from tdd2.pose_frames import PoseFrames
from tdd2.pose_handler import PoseHandler


class YoloLocalization(PoseHandler):

    def __init__(self):
        super().__init__()

        self.hfov_ = rospy.get_param("sync/cam0/hfov")
        self.vfov_ = rospy.get_param("sync/cam0/vfov")
        self.height_ = rospy.get_param("sync/cam0/resolution")[1]
        self.width_ = rospy.get_param("sync/cam0/resolution")[0]
        
        self.current_alt_ = 0.0

        self.intrinsics_ = np.eye(3)
        self.setIntrinsics()
        rospy.loginfo("[LOCALIZER] Localizer Initialized")

    def setIntrinsics(self) -> None:
        intrin = rospy.get_param("sync/cam0/intrinsics")
        self.intrinsics_[0][0] = intrin[0]
        self.intrinsics_[1][1] = intrin[1]
        self.intrinsics_[0][2] = intrin[2]
        self.intrinsics_[1][2] = intrin[3]

    def calculatePixelWidth(self, alt : float) -> float:
        img_width = 2 * (alt * math.tan(self.hfov_/2))
        return img_width / self.width_

    def calculatePixelHeight(self, alt : float) -> float:
        img_height = 2 * (alt * math.sqrt(self.vfov_/2))
        return img_height / self.height_

    def convertToCenter(self, coord : Tuple[int, int]) -> Tuple[int, int]:
        center_x = self.width_ // 2
        center_y = self.height_ // 2

        center_coord_x = coord[0] - center_x
        center_coord_y = coord[1] - center_y

        return (center_coord_x, center_coord_y)

    def localToGlobal(self, local_x : float, local_y : float, local : Tuple[float, float], lat : float, lon : float) -> Tuple[float, float, float, float]:
        zone, e, n = LLtoUTM(23, lat, lon)
        det_e = e + local_x
        det_n = n + local_y
        #print("lat: ", lat, " lon: ", lon)
        #print("easting: ", e, " northing: ", n)
        det_lat, det_lon = UTMtoLL(23, det_n, det_e, zone)

        return (det_lat, det_lon, det_e, det_n)


    def localize(self, pixel_coord : Tuple[int, int], lat : float, lon : float) -> PoseFrames:
        """
        @param : pixel_coord - Tuple [int,int] - x,y coordinate of the center of the bounding box from the top left
        """
        #pixel_width = self.calculatePixelWidth(self.altitude_)
        #pixel_height = self.calculatePixelHeight(self.altitude_)

        #pixel_coord = self.convertToCenter(pixel_coord)

        #rel_x = pixel_coord[0] * pixel_width
        #rel_y = pixel_coord[1] * pixel_height

        pc_array = np.array([pixel_coord[0], pixel_coord[1], 1]).T
        #cam_coords = self.intrinsics_ @ pc_array
        #cam_coords = np.append(cam_coords, 1)
        #local_coords = self.extrinsics_ @ cam_coords

        proj_mat = self.intrinsics_ @ self.extrinsics_
        local_coords = proj_mat.inv() @ pc_array

        rel_x = self.current_pose_.position.x - local_coords[0]
        rel_y = self.current_pose_.position.y - local_coords[1]
        
        local_x = local_coords[0]
        local_y = local_coords[1]

        global_lat, global_lon, global_easting, global_northing = self.localToGlobal(rel_x, rel_y, (self.current_pose_.position.x, self.current_pose_.position.y), lat, lon)
        
        return PoseFrames(global_lat, global_lon, global_easting, global_northing, local_x, local_y, rel_x, rel_y)
