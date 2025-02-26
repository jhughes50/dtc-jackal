"""
    Jason Hughes
    January 2025
    
    Get the current gps from the machine
"""

import rospy 
import math
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

from waypoint_nav.converter import LLtoUTM

from scipy.spatial.transform import Rotation

class LocalizationManager:

    def __init__(self) -> None:
        
        self.gps_ = NavSatFix()
        self.euler_ = list()
        self.heading_ = float()
        self.orientation_ = tuple()
        self.pose_ = Point()
        self.pose_.x = 0.0
        self.pose_.y = 0.0
        rospy.Subscriber("/ublox/fix", NavSatFix, self.gpsCallback)
        rospy.Subscriber("/os_node/rofl_odom/pose", PoseStamped, self.odomCallback)

    def gpsCallback(self, msg : NavSatFix) -> None:
        self.gps_ = msg

    def odomCallback(self, msg : PoseStamped) -> None:
        quat = msg.pose.orientation
        r = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        self.pose_ = msg.pose.position
        self.euler_ = r.as_euler('xyz')
        self.heading_ = self.euler_[-1]

    def distance(self, lat : float, lon : float) -> float:
        """ calculate the distance between current position and desired """
        zone, current_easting, current_northing = LLtoUTM(23, self.gps_.latitude, self.gps_.longitude)
        zone, desired_easting, desired_northing = LLtoUTM(23, lat, lon)

        return ((desired_easting - current_easting) ** 2 + (desired_northing - current_northing) ** 2) ** 0.5

    def localDistance(self, x : float, y : float) -> float:
        return ((x - self.pose_.x) ** 2 + (y - self.pose_.y) ** 2) ** 0.5

    def angle(self, lat : float, lon : float) -> float:
        """ calculate the distance between current position and desired """
        zone, current_easting, current_northing = LLtoUTM(23, self.gps_.latitude, self.gps_.longitude)
        zone, desired_easting, desired_northing = LLtoUTM(23, lat, lon)
    
        de = desired_easting - current_easting
        dn = desired_northing - current_northing

        return math.atan2(dn, de) - self.heading_

    def localAngle(self, x : float, y : float) -> float:
        dx = x - self.pose_.x
        dy = y - self.pose_.y

        return math.atan2(dx, dy) - self.heading_
    
    def waypointAngle(self, x_next : float, y_next : float) -> float:
        dx = self.pose_.x - x_next
        dy = self.pose_.y - y_next

        angle = math.degrees(math.atan2(dx, dy))
        return angle
        
    def twoPointAngle(self, xp : float, yp : float, xn : float, yn : float) -> float:
        dx = xp - xn
        dy = yp - yn

        return math.degrees(math.atan2(dx, dy))

    @property
    def heading(self) -> float:
        return math.degrees(self.heading_)

    @property
    def gps(self) -> NavSatFix:
        return self.gps_
