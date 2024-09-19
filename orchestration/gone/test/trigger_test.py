"""
    Jason Hughes
    September 2024
    Test node for trigger and GroundDetection msg

    DTC PRONTO 2024
"""

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import CompressedImage
from GONe.msg import GroundDetection

class JackalTestNode:

    def __init__(self):

        rospy.Subscriber("/trigger", Bool , self.triggerCallback)
        rospy.Subscriber("camera/image_color/compressed", CompressedImage, self.imageCallback)

        self.detection_pub_ = rospy.Publisher("/ground_detection", GroundDetection, queue_size=2)

        self.trigger_ = False

        rospy.loginfo("[GD-TEST] Initialized Ground Detection Test Node")

    def triggerCallback(self, msg : Bool) -> None:
        rospy.loginfo("[GD-TEST] Trigger Recieved")
        self.trigger_ = msg.data

    def imageCallback(self, msg : CompressedImage) -> None:
        if self.trigger_:
            
            gd = GroundDetection()

            gd.header.stamp = msg.header.stamp
            gd.header.frame_id = os.environ('ROBOT')
            gd.image = msg

            gd.gps.latitude = 39.941287
            gd.gps.longitude = -75.198836

            gd.whisper.data = "whisper whisper whisper"
        
            rospy.loginfo("[GD-TEST] Publishing Detection")
            self.detection_pub_.publish(gd)

if __name__ == "__main__":
    rospy.init_node("ground_detection_test_node")

    JackalTestNode()

    rospy.spin()
