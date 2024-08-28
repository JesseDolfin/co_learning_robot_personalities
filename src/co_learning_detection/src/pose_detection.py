#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Header
from cv_bridge import CvBridge, CvBridgeError
import cv2

import mediapipe as mp
from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2

import numpy as np
import pyrealsense2 as rs
import os
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
        rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_image_callback)

        self.pose_pub = rospy.Publisher('/hand_pose', hand_pose, queue_size=10)

        self.mode = False
        self.maxHands = 1
        self.modelComplex = 1
        self.detectionCon = 0.5
        self.trackCon = 0.5

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode,self.maxHands,self.modelComplex,self.detectionCon,self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

        self.wrist_pixel = None
        self.pose = None
        self.intrinsics = None
        self.rate = rospy.Rate(1)
        self.depth_image = None
        self.rgb_image = None

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
            return
        try:
            # Convert the depth image to a numpy array
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            self.depth_image_time = data.header.stamp
        except CvBridgeError as e:
            rospy.logerr(f"CVBridge Error: {e}")
            return
        
        self.process_image()
        
    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            self.rgb_image_time = data.header.stamp
        except CvBridgeError as e:
            rospy.logerr(f"CVBridge Error: {e}")
            return
          
        self.process_image()

    def findHands(self,img,draw=True):
        img = cv2.flip(img,1)
        imgRGB = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
        #print(results.multi_hand_landmarks)

        if self.results.multi_hand_landmarks:
            for handlms in self.results.multi_hand_landmarks:   
                '''
                
                    '''
                if draw:
                    self.mpDraw.draw_landmarks(img,handlms,self.mpHands.HAND_CONNECTIONS)

        return img
    
    def findPosition(self,img,handNo=0,draw=True):
        
        lmlist = []
        if self.results.multi_hand_landmarks:
            myHand=self.results.multi_hand_landmarks[handNo]

            for id,lm in enumerate(myHand.landmark):
                h,w,c = img.shape
                cx,cy = int(lm.x*w),int(lm.y*h)
                #print(id,cx,cy)
                lmlist.append([id,cx,cy])
                if draw:
                    cv2.circle(img,(cx,cy),5,(255,0,255),cv2.FILLED)

        return lmlist
        
        
    def process_image(self):
        if self.rgb_image is None or self.depth_image is None:
            return
     
        # Check if the images have the same timestamp
        if self.rgb_image_time != self.depth_image_time:
            return
        
        hand = self.findHands(self.rgb_image,draw=True)
        cv2.imshow("result",hand)
        cv2.waitKey(0)
        
        hand_pose_msg = hand_pose()
        hand_pose_msg.header = Header()

        position = self.findPosition(self.rgb_image)

        if position:
            pose = self.determine_hand_pose(position)

            wrist = position[0]
            middle_finger_mcp = position[1]
            self.palm = [(wrist[1] + middle_finger_mcp[1])/2,(wrist[2] + middle_finger_mcp[2])/2]

            self.find_3d_palm(pose)
            

    def find_3d_palm(self,pose):
        if self.palm is None:
            rospy.logwarn_once("Wrist pixel coordinates not yet available")
        else:
            try:
                depth_image = self.depth_image
                height, width = depth_image.shape[:2]

                # Check if wrist_pixel is within the bounds of depth_image
                if 0 <= self.wrist_pixel[1] < height and 0 <= self.wrist_pixel[0] < width:
                    # Get the depth value at the wrist pixel
                    depth = depth_image[self.palm[1], self.palm[0]]
                    point = rs.rs2_deproject_pixel_to_point(self.intrinsics, self.palm, depth)

                    hand_pose_msg = hand_pose()
                    hand_pose_msg.header.stamp = rospy.Time.now()
                    hand_pose_msg.header.frame_id = "camera_link"  # Replace with the correct frame ID
                    hand_pose_msg.x = point[0]
                    hand_pose_msg.y = point[1]
                    hand_pose_msg.z = point[2]
                    hand_pose_msg.orientation = pose
                    self.pose_pub.publish(hand_pose_msg)
                else:
                    rospy.logerr_once("Wrist pixel is out of bounds")

            except CvBridgeError as e:
                rospy.logerr(e)
            except TypeError as e:
                rospy.logerr("TypeError: {}".format(e))

        self.rate.sleep()

    def determine_hand_pose(self, hand_landmarks):
        wrist = hand_landmarks[0] 
        index_mcp = hand_landmarks[5] 
        pinky_mcp = hand_landmarks[17] 

        vector1 = np.array([index_mcp[1] - wrist[1], index_mcp[2] - wrist[2]])
        vector2 = np.array([pinky_mcp[1] - wrist[1], pinky_mcp[2] - wrist[2]])

        normal_vector = np.cross(vector1, vector2) # produces a scalar that represents the magnitude of the z-component of the two vectors, hence the sign determines the orientation of the palm

        if normal_vector > 0:
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