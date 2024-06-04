#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp
import numpy as np

class HandPoseNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.pose_pub = rospy.Publisher('/hand_pose', String, queue_size=10)
        self.mp_hands = mp.solutions.hands.Hands(static_image_mode=True, max_num_hands=2, min_detection_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils

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
                hand_pose = self.determine_hand_pose(hand_landmarks)
                self.pose_pub.publish(hand_pose)

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


