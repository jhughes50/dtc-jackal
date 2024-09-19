"""
    Jason Hughes
    August 2024

    Orchestrator Node to send to the ground

    DTC PRONTO 2024
"""

import rospy
import yaml
import termcolor
from sensor_msgs.msg import NavSatFix, CompressedImage
from std_msgs.msg import Bool, Int32, Float32, String, UInt8
from apriltag_ros.msg import AprilTagDetectionArray
from gone.services import ServiceModule
from gone.msg import GroundDetection

class Orchestrator:

    def __init__(self) -> None:
        
        self.trigger_ = None
        self.override_ = False
        self.casualty_id_ = None
        self.current_gps_ = NavSatFix()
        self.current_image_ = CompressedImage()

        self.config_ = {}
        self.service_count_ = 0
        self.loadConfig()

        self.mtts_reading_ = 0.0
        self.acconeer_reading_ = 0.0
        self.event_reading_ = 0.0
        self.whisper_text_ = ""

        rate = rospy.get_param("/ground_orchestrator/check_rate")
        self.name_ = rospy.get_param("/robot_name")

        rospy.Subscriber("/ublox/fix", NavSatFix, self.gpsCallback)
        rospy.Subscriber("/camera/image_color/compressed", CompressedImage, self.imageCallback)
        rospy.Subscriber("/jackal_teleop/trigger", UInt8, self.triggerCallback)
        rospy.Subscriber("/override", Bool, self.overrideCallback)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tagCallback)

        rospy.Subscriber("/acconeer/respiration_rate", Float32, self.acconeerReadingCallback)
        rospy.Subscriber("/event/respiration_rate", Float32, self.eventReadingCallback)
        rospy.Subscriber("/heart_rate/model", Float32, self.mttsReadingCallback)
        rospy.Subscriber("/whisperer/text", String, self.whisperCallback)
        rospy.Subscriber("/heart_rate/cv", Float32, self.vhrReadingCallback)
        
        self.detection_pub_ = rospy.Publisher("/ground_detection", GroundDetection, queue_size=2)
        rospy.Timer(rospy.Duration(rate), self.cycleCallback)
        rospy.Timer(rospy.Duration(2.0), self.monitorCallback)

    def loadConfig(self):
        with open("/home/dtc/docker-compose.yml", "r") as file1:
            config = yaml.safe_load(file1)["services"]      

        with open("/home/dtc/sensor_services.yml", "r") as file2:
            runtime = yaml.safe_load(file2)
        r_count = 0 
        for service in runtime["services"]:
            envs = config[service]["environment"]
            for e in envs:
                var, val = e.split("=")
                if var == "RUN":
                    if val == "true" or val == "True":
                        run = True
                        self.service_count_ += 1
                    else:
                        run = False
                    print("[GROUND-MONITOR] Service: ", service, " Running: ", run)
                    self.config_[service] = ServiceModule(service, run)
                    break


    def tagCallback(self, msg : AprilTagDetectionArray) -> None:
        if self.trigger_ == 0 and self.casualty_id_ == None:
            if len(msg.detections) <= 1:
                self.casualty_id_ = msg.detections[0].id[0]
            else:
                for i, d in enumerate(msg.detections):
                    tag_cam_pose = d.pose.pose.pose
                    tag_id = d.id[0]
                    if i == 0:
                        lesst_id = tag_id
                        least_pose = tag_cam_pose
                        continue
                    if tag_cam_pose.z < prev_pose.z:
                        least_id = tag_id
                        least_pose = tag_cam_pose
                self.casualty_id_ = least_id
            print("[GROUND-ORCHESTRATOR] Looking at ID: ", self.casualty_id_)


    def whisperCallback(self, msg : String) -> None:
        if self.trigger_ != 0:
            print("[GROUND-ORCHESTRATOR] Received whisper text")
            self.config_["jackal-whisper"].received = True
            self.config_["jackal-whisper"].reading = msg.data

    def eventReadingCallback(self, msg : Float32) -> None:
        if self.trigger_ != 0:
            print("[GROUND-ORCHESTRATION] Received event reading")
            self.config_["jackal-ebreather"].received = True
            self.config_["jackal-ebreather"].reading = msg.data

    def acconeerReadingCallback(self, msg : Float32) -> None:
        if self.trigger_ != 0:
            print("[GROUND-ORCHESTRATION] Received acconeer reading")
            self.config_["jackal-acconeer"].received = True
            self.config_["jackal-acconeer"].reading = msg.data

    def mttsReadingCallback(self, msg : Float32) -> None:
        if self.trigger_ != 0:
            print("[GROUND-ORCHESATRATION] Received MTTS reading")
            self.config_["jackal-mtts"].received = True
            self.config_["jackal-mtts"].reading = msg.data

    def vhrReadingCallback(self, msg : Float32) -> None:
        if self.trigger_ != 0:
            print("[GROUND-DETECTION] Received VHR reading")
            self.config_["jackal-pyvhr"].recieved = True
            self.config_["jackal-pyvhr"].reading = msg.data

    def imageCallback(self, msg: CompressedImage) -> None:
        if self.trigger_ != 0:
            self.current_image_ = msg

    def gpsCallback(self, msg : NavSatFix) -> None:
        self.current_gps_ = msg

    def triggerCallback(self, msg : UInt8) -> None:
        self.trigger_ = msg.data
        print("[GROUND-MONITOR] trigger: ", self.trigger_)
        if self.trigger_ == 0:
            self.casualty_id_ = None

    def overrideCallback(self, msg : Bool) -> None:
        if msg.data:
            # publish the detections we have.
            self.publish()

    def publish(self) -> None:
        msg = GroundDetection()
        msg.header = self.current_image_.header
        msg.header.frame_id = self.name_
        msg.gps = self.current_gps_
        msg.image = self.current_image_
        msg.casualty_id = self.casualty_id_

        if self.config_["jackal-whisper"].run:
            msg.whisper = self.config_["jackal-whisper"].reading
        if self.config_["jackal-acconeer"].run:
            msg.acconeer_respiration_rate = self.config_["jackal-acconeer"].reading
        if self.config_["jackal-ebreather"].run:
            msg.event_respiration_rate = self.config_["jackal-ebreather"].reading
        if self.config_["jackal-mtts"].run:
            msg.neural_heart_rate = self.config_["jackal-mtts"].reading
        if self.config_["jackal-pyvhr"].run:
            msg.cv_heart_rate = self.config_["jackal-pyvhr"].reading

        self.detection_pub_.publish(msg)

    def cycleCallback(self, call) -> None:
        ready = 0 
        for service, config in self.config_.items():
            if config.run and config.received:
                ready += 1
        
        if ready == self.service_count_:
            print("[GROUND-ORCHESTRATION] PUBLISHING DETECTION")
            self.publish() 

    def monitorCallback(self, call) -> None:
        info_str = "[GROUND-MONITOR] "
     
        for service, config in self.config_.items():
            if config.run and config.received:
                info_str = info_str + " " + service.upper() + termcolor.colored(" ready ", "green")
            elif config.run and not config.received:
                info_str = info_str + " " + service.upper() + termcolor.colored(" not ready ", "red")
            else:
                info_str = info_str + " " + service.upper() + " not running "

        print(info_str)