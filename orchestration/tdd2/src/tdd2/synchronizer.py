"""
    Jason Hughes
    August 2024

    DTC PRONTO 2024

"""

import math
import rospy
import message_filters
import numpy as np
import cv2
from cv_bridge import CvBridge

from typing import Tuple

from sensor_msgs.msg import Image, NavSatFix, Imu, CompressedImage
from vision_msgs.msg import Detection2DArray
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Odometry

from tdd2.gps_conversion import LLtoUTM, UTMtoLL
from tdd2.yolo_localizer import YoloLocalization
from tdd2.pose_handler import PoseHandler
from tdd2.pose_frames import PoseFrames
from tdd2.msg import TDDetection

class SynchronizationHandler(PoseHandler):

    def __init__(self):
        super().__init__()
        self.height_ = rospy.get_param("sync/cam0/resolution")[1]
        self.width_ = rospy.get_param("sync/cam0/resolution")[0]

        self.center_x_ = self.width_ // 2
        self.center_y_ = self.height_ // 2

        self.threshold_ = 1000 #pixels
        self.seen_ids_ = []

        self.bridge = CvBridge()
        
        self.yolo_ = rospy.get_param("yolo")
        self.intrinsics_ = rospy.get_param("/sync/cam0/intrinsics")
        
        print("[SYNCHRONIZER] Running YOLO: ", self.yolo_)

        image_sub = message_filters.Subscriber("/image", Image)
        april_sub = message_filters.Subscriber("/tag_detections", AprilTagDetectionArray)
        yolo_sub  = message_filters.Subscriber("/yolov7/yolov7", Detection2DArray)

        self.localizer_ = YoloLocalization()

        if self.yolo_:
            sync = message_filters.TimeSynchronizer([image_sub, april_sub, yolo_sub],10)
            sync.registerCallback(self.imageDetectionCallback)
            self.total_publisher_ = rospy.Publisher("sync/total_detections", TDDetection, queue_size=10)
        else:
            sync = message_filters.TimeSynchronizer([image_sub, april_sub], 5)
            sync.registerCallback(self.imageTagCallback)
            self.tag_publisher_ = rospy.Publisher("sync/aerial_detections", TDDetection, queue_size=2)


    def imageDetectionSyncCallback(self, image : Image, tags : AprilTagDetectionArray, yolo_detections : Detection2DArray) -> None:
        # do nothing if we can't get an ID
        if len(tag.detections) == 0:
            return
        else:
            for d in tags.detections:    
                tag_cam_pose = d.pose.pose.pose
                tag_id = d.id
                tag_local_pose = self.cameraToLocalPose(tag_cam_pose)
            
                pose = self.localToGlobal(tag_local_pose, self.current_gps_.latitude, self.current_gps_.longitude)

        for d in yolo_detections.detections:
            if d.results[0].id == 0:
                pose = self.localizer_.localize((d.bbox.center.x, d.bbox.center.y), self.current_gps_.latitude, self.current_gps_.longitude)
                yolo_poses.append(pose)
                yolo_bboxes.append(d.bbox)                


    def imageDetectionCallback(self, image : Image, tags : AprilTagDetectionArray, yolo_detections : Detection2DArray) -> None:
        
        # look at april tag detections
        tag_poses = list()
        tag_ids = list()
        for d in tags.detections:
            tag_cam_pose = d.pose.pose.pose
            tag_id = d.id
            tag_local_pose = self.cameraToLocalPose(tag_cam_pose)
        
            pose = self.localToGlobal(tag_local_pose, self.current_gps_.latitude, self.current_gps_.longitude)
            tag_poses.append(pose)
            tag_ids.append(tag_id)
        
        yolo_poses = list()
        yolo_bboxes = list()
        for d in yolo_detections.detections:
            if d.results[0].id == 0:
                print("[SYNC] Calling localizer")
                pose = self.localizer_.localize((d.bbox.center.x, d.bbox.center.y), self.current_gps_.latitude, self.current_gps_.longitude)
                yolo_poses.append(pose)
                yolo_bboxes.append(d.bbox)
        
        if len(yolo_poses) != len(tag_poses):
            print("[SYNC] Mismatch between yolo and tag detections: %s yolo detections, %s tag detections" %(len(yolo_poses), len(tag_poses)))

        elif len(yolo_poses) > 0 and len(tag_poses) > 0 and len(yolo_poses) == len(tag_poses):
        
            print("[SYNC] Got %s yolo detections and %s tag detections" %(len(yolo_poses), len(tag_poses)))
            
            for i in range(len(yolo_poses)):
                dist = self.calculateRelativeDistance(yolo_poses[i].local_pose, tag_poses[i].local_pose)
                if dist < self.threshold_:
                    msg = yolo_poses[i].msg
                    msg.casualty_id = tag_id[i]
                    msg.bbox = yolo_bboxes[i]
                    msg.header = image.header
                    
                    self.sync_publisher_.publish(msg)
                else:
                    print("[SYNC] Not within threshold, got distance of ", dist)


    
    def imageTagCallback(self, image : Image, tags : AprilTagDetectionArray) -> None:
        print("[DEBUG] Got detection") 
        # look at april tag detections
        for d in tags.detections:
            tag_cam_pose = d.pose.pose.pose
            tag_id = d.id[0]
            pixel = self.cameraToPixelCoord(tag_cam_pose)

            if pixel.x < self.center_x_ + self.threshold_ and pixel.x > self.center_x_ - self.threshold_ and pixel.y < self.center_x_ + self.threshold_ and pixel.y > self.center_y_ - self.threshold_:
                if tag_id not in self.seen_ids_:
                    print("[SYNCHRONIZER] Publishing Tag: ", tag_id)
                    self.seen_ids_.append(tag_id)
                    msg = self.globalFromPixel(pixel.x, pixel.y)
                    msg.casualty_id = tag_id
                    msg.header = image.header
                    
                    cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")

                    # Compress the image
                    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                    result, encimg = cv2.imencode('.jpg', cv_image, encode_param)

                    # Create CompressedImage message
                    compressed_msg = CompressedImage()
                    compressed_msg.header = image.header
                    compressed_msg.format = "jpeg"
                    compressed_msg.data = np.array(encimg).tostring()            

                    msg.image = compressed_msg

                    self.tag_publisher_.publish(msg)


    def calculateRelativeDistance(self, pose1 : Pose, pose2: Pose) -> None:
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        dz = pose2.position.z - pose1.position.z

        return math.sqrt(dx**2 + dy**2 + dz**2)

    def cameraToPixelCoord(self, camera_pose : Pose) -> Point:
        x_p = camera_pose.position.x / camera_pose.position.z
        y_p = camera_pose.position.y / camera_pose.position.z

        u = self.intrinsics_[0] * x_p + self.intrinsics_[2]
        v = self.intrinsics_[1] * y_p + self.intrinsics_[3]

        p = Point()
        p.x = u
        p.y = v 
        
        return p

    def cameraToLocalPose(self, pose : Pose) -> Pose:
        pose_array = np.array([pose.position.x, pose.position.y, pose.position.z, 1]).T
        local_array = self.extrinsics_homo_ @ pose_array

        local = Pose()
        local.position.x = local_array[0]
        local.position.y = local_array[1]
        local.position.z = local_array[2]

        return local


    def localToGlobal(self, pose : Pose, lat : float, lon : float) -> Tuple[float, float]:
        # convert local pose to gps
        zone, e, n = LLtoUTM(23, lat, lon)
        tag_e = e + pose.position.x
        tag_n = n + pose.position.y

        local_x = self.current_pose_.position.x + pose.position.x
        local_y = self.current_pose_.position.y + pose.position.y

        lat, lon = UTMtoLL(23, tag_n, tag_e, zone)

        return PoseFrames(lat, lon, tag_e, tag_n, local_x, local_y, pose.position.x, pose.position.y)

    def getNorthRelativePose(self, rel_x : int, rel_y : int) -> Tuple[float, float]:
        r = math.sqrt(rel_x**2 + rel_y**2)
        theta = math.atan(rel_y/rel_x)

        theta_p = theta - math.radians(self.current_heading_)
        
        rel_x_p = r * math.cos(theta_p)
        rel_y_p = r * math.sin(theta_p)

        return rel_x_p, rel_y_p


    def globalFromPixel(self, u : int, v : int) -> TDDetection:
        # convert to center
        center_x = self.width_ // 2
        center_y = self.height_ // 2

        center_coord_x = u - center_x
        center_coord_y = v - center_y

        # get relative pose
        relative_x = center_coord_x * self.resolution_
        relative_y = center_coord_y * self.resolution_

        # get current utm
        zone, e, n = LLtoUTM(23, self.current_gps_.latitude, self.current_gps_.longitude)
        
        north_rel_x, north_rel_y = self.getNorthRelativePose(relative_x, relative_y)

        tag_e = north_rel_x + e
        tag_n = north_rel_y + n
        
        lat, lon = UTMtoLL(23, tag_n, tag_e, zone)

        msg = TDDetection()
        msg.easting = tag_e
        msg.northing = tag_n
        
        msg.gps.latitude = lat
        msg.gps.longitude = lon

        msg.pixel.x = center_coord_x
        msg.pixel.y = center_coord_y
        
        msg.relative.x = relative_x
        msg.relative.y = relative_y
            
        return msg


