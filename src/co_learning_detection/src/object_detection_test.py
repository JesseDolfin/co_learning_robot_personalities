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
        results, annotated_image = detector.run_object_detection(frame, visualise=True)

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
        color_frame, _ = detector.get_frames(aligned=True)

        numpy_frame = np.asanyarray(color_frame.get_data())

        img_size = 300

        x_offset = 190
        y_offset = 440

        numpy_frame = numpy_frame[x_offset:img_size+x_offset,y_offset:img_size+y_offset,:]

        numpy_frame = numpy_frame.astype(np.uint8)

        # Run object detection on the RGB frame
        results, annotated_image = detector.run_object_detection(numpy_frame, visualise=True)

        # Display the annotated frame in a window called 'webcam'
        cv2.imshow('realsense image', annotated_image)

        # Exit the loop if the 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


