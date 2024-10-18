"""
    Jason Hughes
    Septemeber 2024
    
    A node to remap an image and
    synchronize the camera info topic

    DTC PRONTO 2024

"""

import math
import yaml
import rospy
import pandas as pd

from sensor_msgs.msg import Image, CameraInfo
from tdd2.msg import TDDetection
from tdd2.gps_conversion import LLtoUTM


class AMMBagTestNode:

    def __init__(self) -> None:
        rospy.Subscriber("/ovc/rgb/image_raw/rect", Image, self.imageCallback)
        rospy.Subscriber("/sync/detections", TDDetection, self.detectionCallback)
        
        self.info_msg_ = self.constructMsg()
        
        self.img_pub_ = rospy.Publisher("/sync/ovc/image_raw", Image, queue_size=1)
        self.info_pub_ = rospy.Publisher("/sync/ovc/camera_info", CameraInfo, queue_size=1)

        self.df_ = pd.read_csv("/home/jasonah/ws/src/tdd2/test/run2.csv")

    def constructMsg(self) -> CameraInfo:
        with open("/home/jasonah/ws/src/tdd2/config/ovc_ros.yaml") as f:
            try:
                data = yaml.safe_load(f)
            except yaml.YAMLError as exc:
                print(exc)

        msg = CameraInfo()

        msg.height = data["image_height"]
        msg.width = data["image_width"]
        msg.distortion_model = data["distortion_model"]
        msg.D = data["distortion_coefficients"]["data"]
        msg.K = data["camera_matrix"]["data"]
        msg.R = data["rectification_matrix"]["data"]
        msg.P = data["projection_matrix"]["data"]

        return msg

    def imageCallback(self, msg : Image) -> None:
        
        self.info_msg_.header = msg.header

        self.img_pub_.publish(msg)
        self.info_pub_.publish(self.info_msg_)

    def detectionCallback(self, msg : TDDetection) -> None:
        
        tag = msg.casualty_id
        row = self.df_[self.df_["casualty_id"] == tag]
        zone, e, n = LLtoUTM(23,row["lat"].item(), row["lon"].item())
        
        d = self.distance(e, n, msg.easting, msg.northing)

        print("[BAG-TEST] Distance %s, with pixel (%s, %s)" %(d, int(msg.pixel.x), int(msg.pixel.y)))

    def distance(self, e1 : float, n1 : float, e2 : float, n2 : float) -> float:
        return math.sqrt((e2 - e1)**2 + (n2 - n1)**2)

if __name__ == "__main__":
    rospy.init_node("ovc_node")
    
    AMMBagTestNode()

    rospy.spin()
