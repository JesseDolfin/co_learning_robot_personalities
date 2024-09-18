import cv2
import numpy as np
from detection import MPDetector  # Assuming you have MPDetector in pose_detection.py



fake = False

# Initialize MPDetector
detector = MPDetector(fake=fake)

if fake:
    # Try opening the camera with the appropriate index
    cam = cv2.VideoCapture(0)

    # Check if the camera is opened correctly
    if not cam.isOpened():
        print('Cannot open camera')
        exit()

    # Ensure the camera has captured an image successfully
    ret, _ = cam.read()
    print('cam has image: %s' % ret)  # True if an image is captured, False otherwise

    # Main loop to process each frame
    while True:
        ret, frame = cam.read()

        if not ret:
            print("Failed to capture image")
            break

        # Run object detection on the RGB frame
        results, annotated_image = detector.run_object_detection(frame, visualize=True)

        # Display the annotated frame in a window called 'webcam'
        cv2.imshow('webcam', annotated_image)

        # Exit the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the camera and close the OpenCV windows
    cam.release()
    cv2.destroyAllWindows()
else:
    while True:
        detector.run_object_detection(visualize=True)

   


