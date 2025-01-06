#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import copy
import cv2
import rospy
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from co_learning_messages.msg import hand_pose, secondary_task_message


class MPDetector:
    def __init__(self, width=1280, height=720, fps=30, fake=False):
        """
        This node takes the camera image from the RealSense camera and uses MediaPipe to determine hand markers.
        These markers are used to determine the orientation of the hand ("serve" vs "drop") by checking if the palm vector points towards or away from the camera.
        The palm marker pixel is then used to determine the depth from the depth image and transforms this into a pose x, y, z in the camera frame.

        Published to:
        - /hand_pose

        hand_pose uses a custom message definition:
        std_msgs/Header header

        (In camera frame, meters)
        float32 x
        float32 y
        float32 z
        string orientation ("serve" / "drop")
        """

        self.depth_clipping_distance = 3.0  # meters
        self.width = width
        self.height = height
        self.fps = fps

        rospy.loginfo("setting up realsense")

        self.setup_realsense()
        self.pose_pub = rospy.Publisher('/hand_pose', hand_pose, queue_size=4)

        rospy.loginfo("realsense setup finished")

        rospy.loginfo("initializing mediapipe")

        self.setup_mediapipe()

    def setup_mediapipe(self):
        # Setup the hand landmarker model
        self.mode = False
        self.max_hands = 1
        self.model_complexity = 1
        self.detection_confidence = 0.5
        self.tracking_confidence = 0.5

        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            self.mode,
            self.max_hands,
            self.model_complexity,
            self.detection_confidence,
            self.tracking_confidence,
        )
        self.mp_draw = mp.solutions.drawing_utils

        # Set the path to your model
        model_path = '/home/worker-20/jesse/ws/src/co_learning_robot_personalities/co_learning_detection/src/exported_model/efficientdet_lite2.tflite'
        model_path_2 = 'src/co_learning_detection/src/exported_model/efficientdet_lite2.tflite' 
        
        try:
            # Setup the detection model
            base_options = python.BaseOptions(model_asset_path=model_path)
            options = vision.ObjectDetectorOptions(
                base_options=base_options, score_threshold=0.10
            )
            self.object_detector = vision.ObjectDetector.create_from_options(options)
        except Exception as e:
            rospy.logwarn(e)

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
        """
        Convert pixel coordinates to a 3D point in the world frame.

        Args:
            pixel_coords (tuple): A tuple (x, y) representing pixel coordinates.
            depth_frame (rs.depth_frame): A RealSense depth frame.

        Returns:
            numpy.ndarray: A 3D point as an (x, y, z) numpy array, or None if an error occurred.
        """
        if not isinstance(depth_frame, rs.depth_frame):
            print("Invalid depth frame")
            return None

        width = depth_frame.get_width()
        height = depth_frame.get_height()

        x, y = pixel_coords
        if x < 0 or x >= width or y < 0 or y >= height:
            print(f"Pixel coordinates {pixel_coords} are out of bounds")
            return None

        try:
            depth_value = depth_frame.get_distance(x, y)

            if depth_value <= 0 or depth_value > self.depth_clipping_distance:
                print(f"Depth value {depth_value} is out of range")
                return None

            depth_point_3d = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [x, y], depth_value)
            return np.array(depth_point_3d)

        except RuntimeError as e:
            print(f"RuntimeError occurred: {e}")
            return None

    def find_hands(self, img, draw=True):
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(img_rgb)

        if self.results.multi_hand_landmarks and draw:
            for hand_landmarks in self.results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(img, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

        return img

    def find_position(self, img, hand_no=0):
        landmark_list = []
        if self.results.multi_hand_landmarks:
            my_hand = self.results.multi_hand_landmarks[hand_no]

            h, w, c = img.shape
            for idx, lm in enumerate(my_hand.landmark):
                cx = int(lm.x * w) + self.y_offset
                cy = int(lm.y * h) + self.x_offset
                landmark_list.append([idx, cx, cy])

        return landmark_list

    def determine_hand_pose(self, hand_landmarks):
        """
        Determines the hand orientation based on landmarks.

        Args:
            hand_landmarks (list): List of hand landmarks.

        Returns:
            str: 'drop' if palm facing away from the camera, 'serve' otherwise.
        """
        wrist = hand_landmarks[0]
        index_mcp = hand_landmarks[5]
        pinky_mcp = hand_landmarks[17]

        vector1 = np.array([index_mcp[1] - wrist[1], index_mcp[2] - wrist[2]])
        vector2 = np.array([pinky_mcp[1] - wrist[1], pinky_mcp[2] - wrist[2]])

        normal_vector = np.cross(vector1, vector2)

        if normal_vector > 0:
            return 'drop'  # Palm facing away from the camera
        else:
            return 'serve'  # Palm facing the camera

    def get_frames(self, aligned=True):
        frames = self.pipeline.wait_for_frames()
        if aligned:
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
        else:
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

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
        if pose != 'none':
            hand_pose_msg.detected = True
        else:
            hand_pose_msg.detected = False
        self.pose_pub.publish(hand_pose_msg)

    def process_frames(self, draw=True):
        try:
            frame_counter = 0
            detection_interval = 30  

            while not rospy.is_shutdown():
                color_frame, depth_frame = self.get_frames()
                if color_frame is None or depth_frame is None:
                    continue

                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                x, y, _ = color_image.shape
                self.x_offset = 300
                self.y_offset = 600
                self.neg_x_offset = 40
                self.neg_y_offset = 250
                x -= self.neg_x_offset
                y -= self.neg_y_offset

                color_image_crop = color_image[self.x_offset:x, self.y_offset:y, :]

                hand_image = self.find_hands(color_image_crop, draw)
                positions = self.find_position(color_image_crop)

                if draw:
                    cv2.imshow("Hand Image", hand_image)
                    cv2.waitKey(1)

                if positions:
                    pose = self.determine_hand_pose(positions)
                    wrist = positions[0]
                    middle_finger_mcp = positions[9]
                    palm = (
                        int((wrist[1] + middle_finger_mcp[1]) / 2),
                        int((wrist[2] + middle_finger_mcp[2]) / 2),
                    )

                    point_3d = self.get_3d_point(palm, depth_frame)

                    if point_3d is not None and point_3d[2] <= 2.433:
                        self.publish_message(pose, point_3d)
                else:
                    point = np.array([0, 0, 0])
                    orientation = 'none'
                    self.publish_message(orientation, point)

                # Commenting out detection code that was causing premature handover success
                # frame_counter += 1
                # if frame_counter >= detection_interval:
                #     rospy.loginfo("Running object detection")
                #     object_detected = self.run_object_detection(visualize=draw)

                #     if self.msg is None:
                #         self.msg = secondary_task_message()

                #     if object_detected:
                #         self.msg.handover_successful = 1
                #         self.secondary_pub.publish(self.msg)
                #     frame_counter = 0

        except rospy.ROSInterruptException:
            pass
        finally:
            self.stop()

    def run_object_detection(self, visualize=False):
        color_frame, _ = self.get_frames(aligned=False)
        if color_frame is None:
            return False

        color_image = np.asanyarray(color_frame.get_data())

        x, y, _ = color_image.shape

        # Smaller workspace configuration
        x_offset = 350
        y_offset = 700
        x_neg_offset = 0
        y_neg_offset = 232

        color_image_crop = color_image[
            x_offset : x - x_neg_offset, y_offset : y - y_neg_offset, :
        ]
        color_image_crop = color_image_crop.astype(np.uint8)

        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=color_image_crop)
        results = self.object_detector.detect(image)

        if visualize:
            bb_image = self.visualize_bb(image, results)
            cv2.imshow('Bounding Box Image', bb_image)
            cv2.waitKey(1)

        for detection in results.detections:
            if any(category.category_name == 'scissors' for category in detection.categories):
                return True

        return False

    def visualize_bb(self, image, results):
        """
        Draws bounding boxes on the input image and returns it.

        Args:
            image: The input RGB image.
            results: The detection results.

        Returns:
            Image with bounding boxes.
        """

        image_np = copy.copy(image.numpy_view())
        margin = 10
        row_size = 10
        font_size = 1
        font_thickness = 1
        text_color = (255, 0, 0)

        for detection in results.detections:
            # Draw bounding box
            bbox = detection.bounding_box
            start_point = (bbox.origin_x, bbox.origin_y)
            end_point = (bbox.origin_x + bbox.width, bbox.origin_y + bbox.height)

            cv2.rectangle(image_np, start_point, end_point, text_color, 3)

            # Draw label and score
            category = detection.categories[0]
            category_name = category.category_name
            probability = round(category.score, 2)
            result_text = f"{category_name} ({probability})"
            text_location = (margin + bbox.origin_x, margin + row_size + bbox.origin_y)
            cv2.putText(
                image_np,
                result_text,
                text_location,
                cv2.FONT_HERSHEY_PLAIN,
                font_size,
                text_color,
                font_thickness,
            )

        return image_np

    def stop(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()


def main():
    rospy.init_node('hand_pose_node')
    hand_pose_detector = MPDetector()
    hand_pose_detector.process_frames()
    rospy.spin()


if __name__ == '__main__':
    main()
