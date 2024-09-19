"""
    Jason Hughes
    August 2024

    Starter for the monitor node

    DTC PRONTO 2024
"""

import rospy
from gone.monitor import DTCGroundMonitor

if __name__ == "__main__":
    rospy.init_node("ground_monitor")
    
    DTCGroundMonitor()

    rospy.spin()
