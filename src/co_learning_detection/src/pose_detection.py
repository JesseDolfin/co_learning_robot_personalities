#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Header
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp
import numpy as np
import pyrealsense2 as rs

from co_learning_messages.msg import hand_pose


class HandPoseNode:
    def __init__(self):
        '''
        This node takes the camera image from the realsense camera and uses mediapipe to determine handmarkers.
        These markers are used to determine the orientation of the hand "serve vs drop" which is determined by checking if the palm vector points towards or away from the camera.
        The palm marker pixel is then used to determine the depth from the depth image and transforms this into a pose x,y,z in camera frame.

        Subsricbed to:
        - /camera/aligned_depth_to_color/camera_info
        - /camera/aligned_depth_to_color/image_raw
        - /camera/color/image_raw

        Published to:
        - /hand_pose

        hand_pose uses a custom message definition:
        std_msgs/Header header

        (In camera frame, meters)
        float32 x 
        float32 y
        float32 z
        string orientation ("serve" / "drop")
        '''


        self.bridge = CvBridge()
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.pose_pub = rospy.Publisher('/hand_pose', hand_pose, queue_size=10)
        rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_image_callback)
        self.mp_hands = mp.solutions.hands.Hands(static_image_mode=True, max_num_hands=2, min_detection_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.wrist_pixel = None
        self.pose = None
        self.intrinsics = None

    def camera_info_callback(self, camera_info):
        # Convert the CameraInfo message to pyrealsense2 intrinsics
        self.intrinsics = rs.intrinsics()
        self.intrinsics.width = camera_info.width
        self.intrinsics.height = camera_info.height
        self.intrinsics.ppx = camera_info.K[2]
        self.intrinsics.ppy = camera_info.K[5]
        self.intrinsics.fx = camera_info.K[0]
        self.intrinsics.fy = camera_info.K[4]
        self.intrinsics.model = rs.distortion.none  # Modify if needed
        self.intrinsics.coeffs = camera_info.D

    def depth_image_callback(self, data):
        if self.intrinsics is None:
            rospy.logwarn_once("Camera intrinsics not yet available")
        else:
            rospy.loginfo_once("Camera intrinsics received")

        if self.wrist_pixel is None:
            rospy.logwarn_once("Wrist pixel coordinates not yet available")
        else:
            try:
                # Convert the depth image to a numpy array
                depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
                height, width = depth_image.shape[:2]

                # Check if wrist_pixel is within the bounds of depth_image
                if 0 <= self.wrist_pixel[1] < height and 0 <= self.wrist_pixel[0] < width:
                    # Get the depth value at the wrist pixel
                    depth = depth_image[self.wrist_pixel[1], self.wrist_pixel[0]]

                    # Deproject the pixel to a 3D point
                    wrist_pixel_floats = [float(self.wrist_pixel[0]), float(self.wrist_pixel[1])]
                    point = rs.rs2_deproject_pixel_to_point(self.intrinsics, wrist_pixel_floats, float(depth))

                    if self.pose != 'None':
                        hand_pose_msg = hand_pose()
                        hand_pose_msg.header = Header()
                        hand_pose_msg.header.stamp = rospy.Time.now()
                        hand_pose_msg.header.frame_id = "camera_link"  # Replace with the correct frame ID
                        hand_pose_msg.x = point[0]
                        hand_pose_msg.y = point[1]
                        hand_pose_msg.z = point[2]
                        hand_pose_msg.orientation = self.pose
                        self.pose_pub.publish(hand_pose_msg)
                else:
                    # Handle the case where the wrist pixel is out of bounds
                    rospy.logerr_once("Wrist pixel is out of bounds")

            except CvBridgeError as e:
                rospy.logerr(e)
            except TypeError as e:
                rospy.logerr("TypeError: {}".format(e))



        
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        results = self.mp_hands.process(image_rgb)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(cv_image, hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS)
                self.pose = self.determine_hand_pose(hand_landmarks)

                # Get wrist pixel coordinates
                wrist = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.WRIST]
                self.wrist_pixel = (int(wrist.x * cv_image.shape[1]), int(wrist.y * cv_image.shape[0]))
        else:
            # Only publish 'None' orientation if there are no hands detected
            if self.pose != 'None':
                hand_pose_msg = hand_pose()
                hand_pose_msg.header = Header()
                hand_pose_msg.header.stamp = rospy.Time.now()
                hand_pose_msg.header.frame_id = "camera_link"  # Replace with the correct frame ID
                hand_pose_msg.orientation = 'None'
                self.pose_pub.publish(hand_pose_msg)
                self.pose = 'None'  # Ensure self.pose is set to 'None'

        cv2.imshow('Hand Pose Result', cv_image)
        cv2.waitKey(3)


    def determine_hand_pose(self, hand_landmarks):
        wrist = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.WRIST]
        index_mcp = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.INDEX_FINGER_MCP]
        pinky_mcp = hand_landmarks.landmark[mp.solutions.hands.HandLandmark.PINKY_MCP]

        vector1 = np.array([index_mcp.x - wrist.x, index_mcp.y - wrist.y, index_mcp.z - wrist.z])
        vector2 = np.array([pinky_mcp.x - wrist.x, pinky_mcp.y - wrist.y, pinky_mcp.z - wrist.z])

        normal_vector = np.cross(vector1, vector2)

        if normal_vector[2] > 0:
            return 'drop'  # Palm facing away from the camera
        else:
            return 'serve'  # Palm facing the camera

def main():
    rospy.init_node('hand_pose_node', anonymous=True)
    hand_pose_node = HandPoseNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()