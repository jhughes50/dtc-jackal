import rospy
import math
import actionlib
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class WaypointNavigator:
    def __init__(self):
        rospy.init_node('waypoint_navigator')
        
        # Create action client for move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.Subscriber("/Odometry", Odometry, self.odom_callback)
        # Example waypoints (x, y) coordinates
        self.waypoints = [
            (3.0, 0.0),
            (3.0, 3.0),
            (0.0, 3.0),
        ]
        self.pose = None
        self.orientation = None

    def odom_callback(self, msg):

        self.pose = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation

    def calculate_orientation(self, current_pos, next_pos):
        """Calculate the yaw angle to face the next waypoint"""
        dx = next_pos[0] - current_pos[0]
        dy = next_pos[1] - current_pos[1]
        yaw = math.atan2(dy, dx)
        return yaw
        
    def create_goal(self, x, y, yaw):
        """Create a MoveBaseGoal with position and orientation"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # Set position
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        
        # Set orientation using quaternion
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation = Quaternion(*q)
        
        return goal
        
    def navigate_waypoints(self):
        """Navigate through all waypoints"""
        for i in range(len(self.waypoints)):
            current_pos = self.waypoints[i]
            
            # Calculate orientation towards next waypoint
            if i < len(self.waypoints) - 1:
                next_pos = self.waypoints[i + 1]
                yaw = self.calculate_orientation(current_pos, next_pos)
            else:
                # For the last waypoint, keep the orientation from the previous movement
                yaw = self.calculate_orientation(self.waypoints[i-1], current_pos)
            print(current_pos)
            # Create and send goal
            goal = self.create_goal(current_pos[0], current_pos[1], yaw)
            rospy.loginfo(f"Navigating to waypoint {i+1}: ({current_pos[0]}, {current_pos[1]})")
            
            # Send goal and wait for result
            self.client.send_goal(goal)
            print("waiting for result")
            self.client.wait_for_result()
            
            # Check if goal was successful
            if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Reached waypoint {i+1}")
            else:
                rospy.logwarn(f"Failed to reach waypoint {i+1}")
                
if __name__ == '__main__':
    try:
        navigator = WaypointNavigator()
        navigator.navigate_waypoints()
    except rospy.ROSInterruptException:
        pass
Last edited just now
