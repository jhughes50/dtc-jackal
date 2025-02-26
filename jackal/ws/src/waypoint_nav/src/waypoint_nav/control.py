"""
    Jason Hughes
    January 2025

    Control loop
"""
import math
import copy
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from waypoint_nav.plan_reader import WaypointPlan
from waypoint_nav.localizer import LocalizationManager

class WaypointController:

    def __init__(self, path : str) -> None:
        
        self.is_auto_ = False

        if path.split('.')[-1] == "json":
            self.local_mode_ = True
            print("[WAAYPOINT-NAV] Initializing in Local Mode")
        elif path.split('.')[-1] == "kml":
            self.local_mode_ = False
            print("[WAYPOINT-NAV] Initializing in Global Mode")

        self.starting_heading_ = 0.0

        self.localizer_ = LocalizationManager()
        self.plan_ = WaypointPlan(path)

        if self.local_mode_:
            self.current_wp_ = self.plan_.getNextLocal()
        else:
            self.current_wp_ = self.plan_.getNextGPS()
        self.prev_wp_ = (0,0)
        # TODO: Make these ros params  
        self.threshold_ = 0.5
        self.angle_threshold_ = 5.0 # degrees
        self.max_linear_ = 2.0
        self.max_angular_ = 1.57
        
        rospy.Subscriber("/jackal_teleop/is_auto", Bool, self.autoCallback)

        self.twist_pub_ = rospy.Publisher("/jackal_velocity_controller/cmd_vel", Twist, queue_size=1)

        self.angle_diff_ = 0.0
        self.traverse_ = True
        self.turn_ = False

        rospy.Timer(rospy.Duration(0.02), self.controlCallback) # 50 hz
        rospy.Timer(rospy.Duration(0.05), self.checkCallback) # 2 hz

    def autoCallback(self, msg : Bool) -> None:
        self.is_auto_ = msg.data
        print("[WAYPOINT-NAV] Auto Mode: ", self.is_auto_)

    def normalizeAngle(self, angle : float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def controlCallback(self, call) -> None:
        """ loop to publish twist msg """
        #print("in callback")
        if self.local_mode_ and self.current_wp_ != None:
            distance = self.localizer_.localDistance(self.current_wp_[0], self.current_wp_[1])
            self.angle_diff_ = self.localizer_.localAngle(self.current_wp_[0], self.current_wp_[1])
        elif self.current_wp_ != None:
            distance = self.localizer_.distance(self.current_wp_[0], self.current_wp_[1])
            self.angle_diff_ = self.localizer_.angle(self.current_wp_[0], self.current_wp_[1])

        #self.angle_diff_ = self.normalizeAngle(self.angle_diff_)

        if self.traverse_:
            linear_velocity = 0.5 #self.max_linear_ * math.cos(self.angle_diff_) * min(distance, 1.0)
            angular_velocity = 0.0
        elif self.turn_: 
            angular_velocity = 0.5 #self.max_angular_ * self.angle_diff_ / math.pi
            linear_velocity = 0.0

        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        
        if self.is_auto_:
            self.twist_pub_.publish(msg)


    def checkCallback(self, call) -> None:
        """ loop to check waypoint """
        #print("heading: ", self.localizer_.heading)
        #print("location: ", self.localizer_.pose_)
        if self.local_mode_:
            if self.traverse_:
                distance = self.localizer_.localDistance(self.current_wp_[1], self.current_wp_[0])
                print("Distance: ", distance)
                if self.localizer_.localDistance(self.current_wp_[1], self.current_wp_[0]) < self.threshold_:
                    print("[WAYPOINT-NAV] waypoint reached ", self.current_wp_)
                    self.prev_wp_ = copy.copy(self.current_wp_)
                    self.threshold_ *= 2
                    self.current_wp_ = self.plan_.getNextLocal()
                    self.traverse_ = False
                    self.turn_ = True
            elif self.turn_:
                #angle = self.localizer_.waypointAngle(self.current_wp_[0], self.current_wp_[1])
                angle = self.localizer_.twoPointAngle(self.prev_wp_[0], self.prev_wp_[1], self.current_wp_[0], self.current_wp_[1])
                print("desired angle : ",angle)
                if (abs(angle - self.localizer_.heading)) < self.angle_threshold_:
                    print("[WAYPOINT-NAV] Facing Next Waypoint")
                    self.traverse_ = True
                    self.turn_ = False 
        else:
            if self.traverse_:
                if self.localizer_.distance(self.current_wp_[1], self.current_wp_[0]) < self.threshold_:
                    print("[WAYPOINT-NAV] waypoint reached", self.current_wp_)
                    self.current_wp_ = self.plan_.getNextGPS()
                    self.traverse_ = False
                    self.turn_ = True
            elif self.turn_:
                if (self.angle_diff_-1.57) < self.angle_threshold_:
                    print("[WAYPOINT-NAV] Facing next waypoint")
                    self.traverse_ = True
                    self.turn_ = False 
        #print("[Waypoint-nav] traverse %s turn %s" %(self.traverse_, self.turn_))
        #print('[waypoint-nav] Current waypoint: ', self.current_wp_)
        if self.current_wp_ == None:
            exit()
