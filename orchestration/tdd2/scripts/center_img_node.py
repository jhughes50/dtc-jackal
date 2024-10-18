#!/usr/bin/python3

import rospy
from tdd2.msg import TDDetection
import cv2

class DatabaseNode:
    def __init__(self):
        rospy.init_node('database_node', anonymous=True)

        self.threshold = rospy.get_param('~threshold', 50)

        self.pub = rospy.Publisher("/centered", TDDetection, queue_size=1)
        self.sub = rospy.Subscriber("/sync/detections", TDDetection, self.callback)

        rospy.loginfo(f"Node started. Threshold: {self.threshold}")

    def callback(self, msg):
        ID = msg.casualty_id

        img_h, img_w = msg.image.height, msg.image.width

        img_center_x = img_w // 2
        img_center_y = img_h // 2

        apriltag_x, apriltag_y = msg.pixel.x, msg.pixel.y

        if abs(apriltag_x - img_center_x) < self.threshold and abs(apriltag_y - img_center_y) < self.threshold:
            rospy.loginfo(f"Publishing data for person ID: {ID}")

            self.pub.publish(msg)

        else:
            rospy.loginfo(f"Person with ID: {ID} is not within center of whole image.")


    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = DatabaseNode()
        node.run()

    except rospy.ROSInterruptException:
        pass
