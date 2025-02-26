"""
    Jason Hughes
    January 2024

    waypoint navigation started script
"""

import rospy
from waypoint_nav.control import WaypointController

if __name__ == "__main__":
    rospy.init_node("waypoint_nav")

    WaypointController("/home/dtc/pennov.json")

    rospy.spin()
