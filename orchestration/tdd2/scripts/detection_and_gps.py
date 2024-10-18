import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, NavSatFix, Imu
from vision_msgs.msg import Detection2DArray
from apriltag_ros.msg import AprilTagDetectionArray
import cv2
import numpy as np
import tf.transformations as tf_trans
import threading

quad_gps = None
quad_orientation = None
bridge = CvBridge()
image_pub = None
capturing = False
video_writer = None
video_file = "uav_output.mp4"
capture_duration = 5 * 60  # t to record stuff


# GPS
def gps_callback(data):
    global quad_gps
    quad_gps = (data.latitude, data.longitude, data.altitude)


# IMU quaternions
def imu_callback(data):
    global quad_orientation
    quad_orientation = (
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w
    )


# YOLO Detection
def yolo_callback(data):
    if not capturing:
        return

    for detection in data.detections:
        if detection.results[0].id == 0:  # classid = 0 is 'person'
            bbox = detection.bbox.center
            class_id = detection.results[0].id
            confidence = detection.results[0].score


# AprilTag Detection
def apriltag_callback(data):
    if not capturing:
        return

    for detection in data.detections:
        tag_id = detection.id[0]    # apriltag id
        pose = detection.pose.pose  # pose of april



# process video
def image_callback(data):
    global image_pub, capturing, video_writer

    if not capturing:
        return

    try:
        frame = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

    except CvBridgeError as e:
        print(e)
        return

    # apriltag detection
    tags = detect_apriltag(frame, at_detector)
    for tag in tags:
        rot, trans = est_tag_pose(tag, camera_mat, dist_coeff, tag_size)

        if rot is not None and trans is not None: # valid apriltag
            trans = trans.flatten()
            tag_gps = pos_to_gps(trans, quad_gps, quad_orientation) # get gps of apriltag from uav

            # draw tag corners and axes on image
            cv2.polylines(frame, [np.int32(tag.corners)], True, (0, 255, 0), 4)
            cv2.drawFrameAxes(frame, camera_mat, dist_coeff, rot, trans, 0.1)

            # display id and stuff
            text = f"Tag {tag.tag_id} | GPS: {tag_gps[0]:.6f}, {tag_gps[1]:.6f}"
            cv2.putText(frame, text, (int(tag.corners[0][0]), int(tag.corners[0][1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # person detection
    person_boxes = detect_people(frame, yolo_model)
    for box in person_boxes:
        x1, y1, x2, y2 = box[:4].astype(int)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2) # draw the box on the detected person

    # save frame immediately to video
    if video_writer is None:
        height, width, _ = frame.shape
        video_writer = cv2.VideoWriter(video_file, cv2.VideoWriter_fourcc(*'mp4v'), 20, (width, height))

    video_writer.write(frame)

    # publish frame for remote viewing if we want
    try:
        image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
    except CvBridgeError as e:
        print(e)


# apriltag detect fx
def detect_apriltag(frame, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray)
    return tags


# apriltag pose
def est_tag_pose(tag, camera_mat, dist_coeff, tag_size):
    corners = np.array([tag.corners], dtype=np.float32)
    tag_points = np.array([
        [-tag_size / 2, -tag_size / 2, 0],
        [tag_size / 2, -tag_size / 2, 0],
        [tag_size / 2, tag_size / 2, 0],
        [-tag_size / 2, tag_size / 2, 0]
    ])
    _, rot, trans = cv2.solvePnP(tag_points, corners, camera_mat, dist_coeff)
    return rot, trans


# rotation correction of position of uav
def apply_rotation(position, orientation):
    # Convert quaternion to rotation matrix
    rot_matrix = tf_trans.quaternion_matrix(orientation)[:3, :3]
    rotated_position = np.dot(rot_matrix, position)
    return rotated_position


# pos apriltag to gps
def pos_to_gps(pos_rel, gps_quad, orient_quad):
    x_rel, y_rel, z_rel = apply_rotation(pos_rel, orient_quad)
    lat_quad, long_quad, alt_quad = gps_quad
    gps_conv = 1e-5  # Conversion factor for meters to degrees
    lat_global = lat_quad + gps_conv * x_rel
    long_global = long_quad + gps_conv * y_rel
    alt_global = alt_quad + z_rel
    return lat_global, long_global, alt_global


# detect people YOLOv7
def detect_people(frame, yolo):
    img_rgb = frame[:, :, ::-1]  # Convert to RGB before passing to Yolov7
    boxes = yolo(img_rgb, conf=0.50, iou=0.45)
    boxes = boxes.cpu().numpy()
    # Filter to only return boxes for 'person' class (COCO class_id = 0)
    person_boxes = [box for box in boxes if box[5] == 0]  # COCO class_id = 0 is 'person'
    return np.array(person_boxes)


# Start/Stop capture video
def start_capture():
    global capturing
    capturing = True
    rospy.loginfo("Started capturing images.")
    threading.Timer(capture_duration, stop_capture).start()


def stop_capture():
    global capturing, video_writer
    capturing = False
    rospy.loginfo("Stopped capturing images.")
    if video_writer is not None:
        video_writer.release()
        rospy.loginfo(f"Video saved as {video_file}")
        video_writer = None


# Main Function
def main():
    global image_pub, at_detector, yolo_model, camera_mat, dist_coeff, tag_size

    rospy.init_node('uav_stuff', anonymous=True)

    image_pub = rospy.Publisher('/uav/processed_image', Image, queue_size=10)

    rospy.Subscriber('/mavros/global_position/global', NavSatFix, gps_callback)
    rospy.Subscriber('/mavros/imu', Imu, imu_callback)
    rospy.Subscriber('/yolov7/out_topic', Detection2DArray, yolo_callback)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, apriltag_callback)
    rospy.Subscriber('/camera/image_mono', Image, image_callback)

    # Initialize detectors and models
    at_detector = April_Detector()
    yolo_model = initialize_yolov7()

    camera_mat = np.array([[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32) # change camera mat and stuff
    dist_coeff = np.zeros((5, 1), dtype=np.float32)
    tag_size = 0.2

    start_capture()

    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
