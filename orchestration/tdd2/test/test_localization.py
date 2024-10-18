"""
    Jason Hughes
    August 2024

    Test script to verify that the 
    localization from the apriltag 
    and yolo are close enough

    DTC PRONTO 2024
"""

import math
import rospy
import pandas as pd

from geometry_msgs.msg import Pose
from tdd2.msg import TDDetection

from tdd2.gps_conversion import LLtoUTM, UTMtoLL

class LocalizationTestNode:

    def __init__(self) -> None:
        
        rospy.Subscriber("/debug/tag_pose", TDDetection, self.tagCallback)
        
        self.ground_truth_ = pd.read_csv("/home/jasonah/ws/src/tdd2/test/run2.csv")

    def tagCallback(self, msg : TDDetection) -> None:
        c_id = msg.casualty_id
        row = self.ground_truth_.loc[self.ground_truth_['casualty_id'] == c_id]

        gt_lat = row.get('lat').item()
        gt_lon = row.get('lon').item()
        #print(gt_lat, type(gt_lat))
        zone, gt_e, gt_n = LLtoUTM(23, gt_lat, gt_lon)

        d = self.distance(gt_e, gt_n, msg.easting, msg.northing)

        print("[TEST-LOCALIZATION] Casualty ID: %s  Distance: %s" %(c_id, d))

    def distance(self, e1 : float, n1 : float, e2 : float, n2 : float) -> float:
        de = e2 - e1
        dn = n2 - n1

        return math.sqrt(de**2 + dn**2)

if __name__ == "__main__":
    rospy.init_node("localization_test_node")

    ltn = LocalizationTestNode()

    rospy.spin()
