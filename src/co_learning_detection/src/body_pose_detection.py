from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np
import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pyrealsense2 as rs


class BodyPose():
    def __init__(self) -> None:
        self.bridge = CvBridge()
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_image_callback)

        base_options = python.BaseOptions(model_asset_path='pose_landmarker_heavy.task')
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            output_segmentation_masks=True)
        self.detector = vision.PoseLandmarker.create_from_options(options)

        self.rgb_image = None
        self.depth_image = None
        self.rgb_image_time = 0 
        self.depth_image_time = 0

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


    def draw_landmarks_on_image(self,rgb_image, detection_result):
        pose_landmarks_list = detection_result.pose_landmarks
        annotated_image = np.copy(rgb_image)

        # Loop through the detected poses to visualize.
        for idx in range(len(pose_landmarks_list)):
            pose_landmarks = pose_landmarks_list[idx]

            # Draw the pose landmarks.
            pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            pose_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in pose_landmarks
            ])
            solutions.drawing_utils.draw_landmarks(
            annotated_image,
            pose_landmarks_proto,
            solutions.pose.POSE_CONNECTIONS,
            solutions.drawing_styles.get_default_pose_landmarks_style())
        return annotated_image
    
    def process_image(self):
        if self.rgb_image is None or self.depth_image is None:
            return
     
        # Check if the images have the same timestamp
        if self.rgb_image_time != self.depth_image_time:
            return
            
        # STEP 3: Load the input image.
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=self.rgb_image)

        # STEP 4: Detect pose landmarks from the input image.
        detection_result = self.detector.detect(image)
        
        # STEP 5: Process the detection result. In this case, visualize it.
        annotated_image = self.draw_landmarks_on_image(image.numpy_view(), detection_result)
        cv2.imshow("test",annotated_image)
        cv2.waitKey(1)
        
def main():
    rospy.init_node('body_pose_node', anonymous=True)
    body_pose_node = BodyPose()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()