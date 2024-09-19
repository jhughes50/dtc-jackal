"""
    Jason Hughes
    August 2024

    Starter for the ground orchestration node

    DTC PRONTO 2024
"""

import rospy
from gone.orchestrator import Orchestrator

if __name__ == "__main__":
    rospy.init_node("ground_orchestrator")
    
    orchestra = Orchestrator()

    rospy.spin()
