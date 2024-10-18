#!/usr/bin/python3
"""
    Jason Hughes
    August 2024

    DTC PRONTO 2024

"""

import rospy
from tdd2.synchronizer import SynchronizationHandler

if __name__ == "__main__":

    rospy.init_node("sync_node")

    sh = SynchronizationHandler()

    rospy.spin()

