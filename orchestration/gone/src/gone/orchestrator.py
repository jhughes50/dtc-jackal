"""
    Jason Hughes
    August 2024

    Orchestrator Node to send to the ground

    DTC PRONTO 2024
"""

import rospy
import yaml
import termcolor
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import NavSatFix, CompressedImage, Image
from std_msgs.msg import Bool, Int32, Float32, String, UInt8
from apriltag_ros.msg import AprilTagDetectionArray
from gone.services import ServiceModule
from gone.msg import GroundDetection
from gone.msg import GroundImage

class Orchestrator:

    def __init__(self) -> None:
        
        self.trigger_ = None
        self.override_ = False
        self.casualty_id_ = None
        self.current_gps_ = NavSatFix()
        self.current_image_ = CompressedImage()
        self.ground_msg_ = GroundImage()

        self.handler_ = DetectionHandler()

        self.img_stop_ = True
        self.img_count_ = 0
        self.send_3_ = 0

        self.config_ = {}
        self.service_count_ = 0
        self.loadConfig()

        self.bridge_ = CvBridge()

        self.mtts_reading_ = 0.0
        self.acconeer_reading_ = 0.0
        self.event_reading_ = 0.0
        self.mtts_rr_reading_ = 0.0
        self.whisper_text_ = ""

        rate = rospy.get_param("/ground_orchestrator/check_rate")
        self.name_ = rospy.get_param("/robot_name")

        rospy.Subscriber("/ublox/fix", NavSatFix, self.gpsCallback)
        rospy.Subscriber("/camera/image_color", Image, self.imageCallback)
        rospy.Subscriber("/jackal_teleop/trigger", UInt8, self.triggerCallback)
        rospy.Subscriber("/override", Bool, self.overrideCallback)
        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.tagCallback)

        rospy.Subscriber("/acconeer/respiration_rate", Float32, self.acconeerReadingCallback)
        rospy.Subscriber("/respiration_rate/model", Float32, self.mttsRespReadingCallback)
        rospy.Subscriber("/heart_rate/model", Float32, self.mttsReadingCallback)
        rospy.Subscriber("/whisperer/text", String, self.whisperCallback)
        rospy.Subscriber("/heart_rate/pyvhr", Float32, self.vhrReadingCallback)
        
        self.detection_pub_ = rospy.Publisher("/ground_detection", GroundDetection, queue_size=2)
        self.img_pub_ = rospy.Publisher("/ground_image", GroundImage, queue_size=3)

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
            if len(msg.detections) == 0:
                print(termcolor.colored("[GROUND ORCHESTRATION][WARN] ID Needed ", "yellow")) 
                self.casualty_id_ = 0
            elif len(msg.detections) == 1:
                self.casualty_id_ = msg.detections[0].id[0]
            else:
             
                for i, d in enumerate(msg.detections):
                    tag_cam_pose = d.pose.pose.pose
                    tag_id = d.id[0]
                    if i == 0:
                        least_id = tag_id
                        least_pose = tag_cam_pose
                        continue
                    if tag_cam_pose.position.z < least_pose.position.z:
                        least_id = tag_id
                        least_pose = tag_cam_pose
                self.casualty_id_ = least_id
            print("[GROUND-ORCHESTRATOR][INFO] Ready for imaging")


    def whisperCallback(self, msg : String) -> None:
        if self.trigger_ != 0:
            print(termcolor.colored("[GROUND-ORCHESTRATOR][WHISPER] Ready To Publish","green"))
            self.config_["jackal-whisper"].received = True
            self.config_["jackal-whisper"].reading = msg.data

    def eventReadingCallback(self, msg : Float32) -> None:
        if self.trigger_ != 0:
            print(termcolor.colored("[GROUND-ORCHESTRATION] Received event reading","green"))
            self.config_["jackal-ebreather"].received = True
            self.config_["jackal-ebreather"].reading = msg.data

    def acconeerReadingCallback(self, msg : Float32) -> None:
        if self.trigger_ != 0:
            print(termcolor.colored("[GROUND-ORCHESTRATION][ACCONEER] Ready To Publish","green"))
            self.config_["jackal-acconeer"].received = True
            self.config_["jackal-acconeer"].reading = msg.data

    def mttsRespReadingCallback(self, msg : Float32) -> None:
        if self.trigger_ != 0:
            print(termcolor.colored("[GROUND-ORCHESATRATION][MTTS-HR] Ready To Publish","green"))
            self.mtts_rr_reading_ = msg.data

    def mttsReadingCallback(self, msg : Float32) -> None:
        if self.trigger_ != 0:
            print(termcolor.colored("[GROUND-ORCHESATRATION][MTTS-RR] Ready To Publish","green"))
            self.config_["jackal-mtts"].received = True
            self.config_["jackal-mtts"].reading = msg.data

    def vhrReadingCallback(self, msg : Float32) -> None:
        if self.trigger_ != 0:
            print(termcolor.colored("[GROUND-DETECTION] Received VHR reading","green"))
            self.config_["jackal-pyvhr"].received = True
            self.config_["jackal-pyvhr"].reading = msg.data

    def imageCallback(self, msg: Image) -> None:
        if self.trigger_ == None: return
        if self.trigger_ > 0:
            self.img_count_ += 1
            if (self.img_count_ % 100 == 0) and self.send_3_ < 3:
                self.send_3_ += 1
                img = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                cv_img = cv2.resize(img, (msg.width//2, msg.height//2))
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                result, encimg = cv2.imencode('.jpg', cv_img, encode_param)

                n_msg = CompressedImage()
                n_msg.header = msg.header
                n_msg.header.frame_id = "phobos"
                n_msg.format="jpeg"
                n_msg.data = np.array(encimg).tostring()
                if self.send_3_ == 1:
                    self.ground_msg_.image1 = n_msg
                    self.ground_msg_.casualty_id.data = self.casualty_id_
                elif self.send_3_ == 2:
                    self.ground_msg_.image2 = n_msg
                elif self.send_3_ == 3:
                    self.ground_msg_.image3 = n_msg
                    self.ground_msg_.gps = self.current_gps_
                    self.img_pub_.publish(self.ground_msg_) 
                    print("[GROUND-ORCHESTRATION] Sending images")

    def gpsCallback(self, msg : NavSatFix) -> None:
        self.current_gps_ = msg

    def triggerCallback(self, msg : UInt8) -> None:
        self.trigger_ = msg.data
        print("[GROUND-MONITOR] Trigger: ", self.trigger_)
        self.timer_ = rospy.get_time()
        if self.trigger_ == 0:
            self.casualty_id_ = None
        else:
            self.handler_.trigger = self.trigger_
            self.send_3_ = 0
            self.ground_msg_ = GroundImage()

    def overrideCallback(self, msg : Bool) -> None:
        if msg.data:
            # publish the detections we have.
            self.publish()

    def publish(self) -> None:
        msg = GroundDetection()
        msg.header = self.current_image_.header
        msg.header.frame_id = self.name_
        msg.gps = self.current_gps_
        msg.casualty_id.data = self.casualty_id_

        msg.whisper.data = " "

        if self.config_["jackal-whisper"].run and self.config_["jackal-whisper"].received:
            msg.whisper.data = self.config_["jackal-whisper"].reading
            self.config_["jackal-whisper"].received = False 
        if self.config_["jackal-acconeer"].run:
            msg.acconeer_respiration_rate.data = self.config_["jackal-acconeer"].reading
            self.config_["jackal-acconeer"].received=False
        if self.config_["jackal-ebreather"].run:
            msg.event_respiration_rate.data = self.config_["jackal-ebreather"].reading
            self.config_["jackal-ebreather"].received = False
        if self.config_["jackal-mtts"].run:
            msg.neural_heart_rate.data = self.config_["jackal-mtts"].reading
            msg.event_respiration_rate.data = self.mtts_rr_reading_
            self.config_["jackal-mtts"].received = False
        if self.config_["jackal-pyvhr"].run:
            msg.cv_heart_rate.data = self.config_["jackal-pyvhr"].reading
            self.config_["jackal-pyvhr"].received = False

        self.detection_pub_.publish(msg)

    def cycleCallback(self, call) -> None:
        ready = 0 
        for service, config in self.config_.items():
            if config.run and config.received:
                ready += 1
        
        if ready == self.service_count_:
            print(termcolor.colored("[GROUND-ORCHESTRATION][HEALTHY] Publishing Detections","green"))
            self.handler_.published = True
            self.publish() 
            return
        #print(self.handler_.timer, self.handler_.published)
        if self.handler_.timer > 30 and not self.handler_.published:
            print(termcolor.colored("[GROUND-MONITOR][ERROR] Didn't receive all readings after 25 seconds, moving on.","red"))
            self.handler_.published = True
            self.publish()

    def monitorCallback(self, call) -> None:
        if self.trigger_ == None: return

        if self.trigger_ > 0:
            info_str = "[GROUND-MONITOR] "
         
            for service, config in self.config_.items():
                if config.run and config.received:
                    info_str = info_str + " " + service.upper() + termcolor.colored(" ready ", "green")
                elif config.run and not config.received:
                    info_str = info_str + " " + service.upper() + termcolor.colored(" not ready ", "red")
                else:
                    info_str = info_str + " " + service.upper() + " not running "

            print(info_str)


class DetectionHandler:

    def __init__(self):
        self.MAX_ = 18446744073709551615

        self.published_ = True
        self.trigger_ = 0

        self.time_ = 0.0
        
    @property
    def timer(self):
        now = rospy.get_time()
        return now-self.time_

    @property
    def published(self) -> bool:
        return self.published_

    @published.setter
    def published(self, p : bool) -> None:
        self.published_ = p

    @property
    def trigger(self) -> int:
        return self.trigger_

    @trigger.setter
    def trigger(self, t : int) -> None: 
        self.time_ = rospy.get_time()
        self.published_ = False
