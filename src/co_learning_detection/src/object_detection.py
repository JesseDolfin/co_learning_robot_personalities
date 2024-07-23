#!/usr/bin/env python

from tabnanny import verbose
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
from ultralytics import YOLO
import numpy as np
import pyrealsense2 as rs
import time
from co_learning_messages.msg import Detection, Database

class ObjectDetection:
    def __init__(self):
        # Initialize the CvBridge
        self.bridge = CvBridge()

        # Subscribe to the image and camera info topics
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_image_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.camera_info_callback)

        self.pub = rospy.Publisher('object_database',Database,queue_size=1)

        # Initialize the YOLO model
        self.model = YOLO('yolov10m')
        self.names = self.model.names

        # Camera intrinsics and detections
        self.intrinsics = None
        self.rgb_image = None
        self.depth_image = None
        self.detections = []
        self.last_processed_time = 0  # To track time of last YOLO detection

        self.rate = rospy.Rate(1)

    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.rgb_image_time = data.header.stamp
        except CvBridgeError as e:
            rospy.logerr(f"CVBridge Error: {e}")
            return

        self.process_images()

    def depth_image_callback(self, data):
        if self.intrinsics is None:
            rospy.logwarn_once("Camera intrinsics not yet available")
            return
        try:
            # Convert the depth image to a numpy array
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            self.depth_image_time = data.header.stamp
        except CvBridgeError as e:
            rospy.logerr(f"CVBridge Error: {e}")
            return

        self.process_images()

    def process_images(self):
        if self.rgb_image is None or self.depth_image is None:
            return

        # Check if the images have the same timestamp
        if self.rgb_image_time != self.depth_image_time:
            return

        results = self.model(self.rgb_image,verbose=False)
        self.detections = []  

        for result in results:
            boxes = result.boxes  
            if boxes is not None:
                for box in boxes.data:
                    id = int(box[5].item())
                    x = (box[0] + box[2]) / 2.0
                    y = (box[1] + box[3]) / 2.0
                    self.detections.append({"id": self.names[id], "center": (x, y, 0)})  

        database = Database()
        database.header.stamp = rospy.Time.now()
        database.header.frame_id = 'camera_link'

        for detection in self.detections:
            
            x, y, _ = detection["center"]

            if 0 <= y < self.depth_image.shape[0] and 0 <= x < self.depth_image.shape[1]:
                depth = self.depth_image[int(y), int(x)]
                point = rs.rs2_deproject_pixel_to_point(self.intrinsics, [float(x), float(y)], float(depth))
                detection["center"] = (point[0], point[1], point[2])
                detection_msg = Detection()
                detection_msg.id = detection["id"]
                detection_msg.center = detection['center']
                database.detection.append(detection_msg)
            else:
                rospy.logwarn_once("Detection center is out of depth image bounds")

        self.pub.publish(database)
        self.rate.sleep()

    def camera_info_callback(self, camera_info):
        self.intrinsics = rs.intrinsics()
        self.intrinsics.width = camera_info.width
        self.intrinsics.height = camera_info.height
        self.intrinsics.ppx = camera_info.K[2]
        self.intrinsics.ppy = camera_info.K[5]
        self.intrinsics.fx = camera_info.K[0]
        self.intrinsics.fy = camera_info.K[4]
        self.intrinsics.model = rs.distortion.none  
        self.intrinsics.coeffs = camera_info.D

if __name__ == '__main__':
    rospy.init_node('object_detection', anonymous=True)
    object_detection = ObjectDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
