import pyrealsense2 as rs
import numpy as np
import cv2
import rospy
import mediapipe as mp
from co_learning_messages.msg import hand_pose

class HandPoseDetector():
    def __init__(self, width=1280, height=720, fps=30):
        '''
        This node takes the camera image from the realsense camera and uses mediapipe to determine hand markers.
        These markers are used to determine the orientation of the hand "serve vs drop" which is determined by checking if the palm vector points towards or away from the camera.
        The palm marker pixel is then used to determine the depth from the depth image and transforms this into a pose x,y,z in camera frame.

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

        self.depth_clipping_distance = 3.0 # m
        self.width = width
        self.height = height
        self.fps = fps

        self.setup_realsense()
        self.setup_mediapipe()

        self.wrist_pixel = None
        self.pose = None

        self.pose_pub = rospy.Publisher('/hand_pose', hand_pose, queue_size=4)

    def setup_mediapipe(self):
        self.mode = False
        self.maxHands = 1
        self.modelComplex = 1
        self.detectionCon = 0.5
        self.trackCon = 0.5

        self.mpHands = mp.solutions.hands
        self.hands = self.mpHands.Hands(self.mode, self.maxHands, self.modelComplex, self.detectionCon, self.trackCon)
        self.mpDraw = mp.solutions.drawing_utils

    def setup_realsense(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Configure the pipeline to stream different resolutions of color and depth streams
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)

        # Start streaming
        self.profile = self.pipeline.start(self.config)

        # Getting the depth sensor's depth scale
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = self.depth_sensor.get_depth_scale()

        # Calculate clipping distance in depth units
        self.clipping_distance = self.depth_clipping_distance / self.depth_scale

        # Create an align object
        self.align_to = rs.stream.depth
        self.align = rs.align(self.align_to)

        # Intrinsics and extrinsics
        self.depth_intrin = None
        self.color_intrin = None
        self.color_to_depth_extrin = None
        self.depth_to_color_extrin = None

        self._setup_intrinsics_and_extrinsics()

    def _setup_intrinsics_and_extrinsics(self):
        # Get the depth and color intrinsics and extrinsics
        depth_stream = self.profile.get_stream(rs.stream.depth)
        color_stream = self.profile.get_stream(rs.stream.color)
        self.depth_intrin = depth_stream.as_video_stream_profile().intrinsics
        self.color_intrin = color_stream.as_video_stream_profile().intrinsics

        self.color_to_depth_extrin = color_stream.get_extrinsics_to(depth_stream)
        self.depth_to_color_extrin = depth_stream.get_extrinsics_to(color_stream)

    def get_3d_point(self, pixel_coords, depth_frame):
        # Extract depth value from the depth frame
        depth_value = depth_frame.get_distance(pixel_coords[0], pixel_coords[1])
        if depth_value <= 0 or depth_value > self.depth_clipping_distance:
            return None
        
        # Deproject from pixel to 3D point using camera intrinsics
        depth_point_3d = rs.rs2_deproject_pixel_to_point(self.depth_intrin, pixel_coords, depth_value)

        return np.array(depth_point_3d)
    
    def findHands(self, img, draw=True):
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(imgRGB)
       
        if self.results.multi_hand_landmarks:
            for handlms in self.results.multi_hand_landmarks:   
                if draw:
                    self.mpDraw.draw_landmarks(img, handlms, self.mpHands.HAND_CONNECTIONS)

        return img
    
    def findPosition(self, img, handNo=0, draw=False):
        lmlist = []
        if self.results.multi_hand_landmarks:
            myHand = self.results.multi_hand_landmarks[handNo]

            for id, lm in enumerate(myHand.landmark):
                h, w, c = img.shape
                cx, cy = int(lm.x * w), int(lm.y * h)
                lmlist.append([id, cx, cy])
                if draw:
                    cv2.circle(img, (cx, cy), 5, (255, 0, 255), cv2.FILLED)

        return lmlist
    
    def determine_hand_pose(self, hand_landmarks):
        wrist = hand_landmarks[0] 
        index_mcp = hand_landmarks[5] 
        pinky_mcp = hand_landmarks[17] 

        vector1 = np.array([index_mcp[1] - wrist[1], index_mcp[2] - wrist[2]])
        vector2 = np.array([pinky_mcp[1] - wrist[1], pinky_mcp[2] - wrist[2]])

        normal_vector = np.cross(vector1, vector2) 

        if normal_vector < 0:
            return 'drop'  # Palm facing away from the camera
        else:
            return 'serve'  # Palm facing the camera
    
    def get_frames(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not depth_frame or not color_frame:
            return None, None
        
        return color_frame, depth_frame

    def publish_message(self, pose, point_3d):
        hand_pose_msg = hand_pose()
        hand_pose_msg.header.stamp = rospy.Time.now()
        hand_pose_msg.header.frame_id = "camera_link"
        hand_pose_msg.x = point_3d[0]
        hand_pose_msg.y = point_3d[1]
        hand_pose_msg.z = point_3d[2]
        hand_pose_msg.orientation = pose
        self.pose_pub.publish(hand_pose_msg)

    def process_frames(self, draw=False):
        try:
            while not rospy.is_shutdown():
                color_frame, depth_frame = self.get_frames()

                if draw:
                    depth_image = np.asanyarray(depth_frame.get_data())
                    depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
                    depth_image_normalized = np.uint8(depth_image_normalized)
                    cv2.imshow("Depth Image", depth_image_normalized)

                    color_image = np.asanyarray(color_frame.get_data())
                    hand = self.findHands(color_image)
                    cv2.imshow("Hand Image", hand) 
                    cv2.waitKey(1)

                positions = self.findPosition(color_image, False)

                if positions:
                    pose = self.determine_hand_pose(positions)
                    wrist = positions[0]
                    middle_finger_mcp = positions[9]
                    palm = (int((wrist[1] + middle_finger_mcp[1]) / 2), int((wrist[2] + middle_finger_mcp[2]) / 2))

                    point_3d = self.get_3d_point(palm, depth_frame)
                    
                    if point_3d is not None:
                        self.publish_message(pose, point_3d)

        except rospy.ROSInterruptException:
            pass
        finally:
            self.stop()

    def stop(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()

def main():
    rospy.init_node('hand_pose_node', anonymous=True)
    hand_pose_node = HandPoseDetector()
    hand_pose_node.process_frames(draw=True)  
    rospy.spin()

if __name__ == '__main__':
    main()
