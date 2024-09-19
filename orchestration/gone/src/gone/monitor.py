"""
    Jason Hughes
    September 2024

    A ros node to monitor activate nodes and topics 
    to ensure they are healthy.

    DTC PRONTO 2024
"""

import rospy 
import rosnode
import difflib
from termcolor import colored
from std_msgs.msg import UInt8

class DTCGroundMonitor:

    def __init__(self) -> None:

        self.current_nodes_ = list()
        self.prev_nodes_ = list()

        self.count_ = 0
        self.differ_ = difflib.Differ()

        rospy.Subscriber("/jackal_telop/trigger", UInt8, self.triggerCallback)

        rospy.Timer(rospy.Duration(2.0), self.monitorCallback)

    def triggerCallback(self, msg : UInt8) -> None:
        print("[GROUND-MONITOR] trigger count : ", msg.data)

    def diff(self) -> None:
        diff_list = list(self.differ_.compare(self.prev_nodes_, self.current_nodes_))

        for e in diff_list:
            if e[0] == '-':
                print(colored("[GROUND-MONITOR] LOST NODE: ", 'red'), e)
            elif e[0] == '+':
                print("[GROUND-MONITOR] Saw bringup of: ", e)


    def monitorCallback(self, call) -> None:
        self.current_nodes_ = rosnode.get_node_names()
        if self.count_ == 0:
            self.prev_nodes_ = self.current_nodes_
            self.count_ += 1
        else:
            if len(self.current_nodes_) == len(self.prev_nodes_):
                print("[GROUND-MONITOR] No nodes added or lost")
            else:
                diff_list = self.diff()
            self.prev_nodes_ = self.current_nodes_

