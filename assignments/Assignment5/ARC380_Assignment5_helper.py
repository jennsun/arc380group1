# import time

# import pyrealsense2 as rs
# import numpy as np
# import cv2
# from matplotlib import pyplot as plt
import cv2
import time

def capture_img(path: str = 'img.png'):
    # Initialize the video capture object with the camera index
    cap = cv2.VideoCapture(0)  # Adjust the index if necessary

    if not cap.isOpened():
        print("Cannot open camera")
        exit()

    # Set the desired resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    # Initiate first capture to start autoexposure
    ret, frame = cap.read()

    time.sleep(2)  # Allow the camera to warm up

    # Actual captured image
    ret, frame = cap.read()

    # Check if the frame was captured successfully
    if ret:
        # Display the captured image
        cv2.imshow('Captured Image', frame)
        cv2.waitKey(0)  # Wait for a key press to exit
        cv2.destroyAllWindows()

        # Optionally, save the image to a file
        cv2.imwrite(path, frame)
    else:
        print("Failed to capture frame")

    # Release the video capture object
    cap.release()
    return frame


# def capture_img(visualize: bool = False, save: bool = False, path: str = 'img.png') -> np.ndarray:
#     # Create a pipeline
#     pipeline = rs.pipeline()

#     # Create a config and configure the pipeline to stream
#     config = rs.config()
#     config.enable_stream(rs.stream.color, 1920, 1080, rs.format.bgr8, 30)

#     # Start streaming
#     pipeline.start(config)

#     # Get the device and color sensor
#     profile = pipeline.get_active_profile()
#     device = profile.get_device()
#     color_sensor = device.first_color_sensor()

#     color_sensor.set_option(rs.option.enable_auto_exposure, 1)
#     color_sensor.set_option(rs.option.enable_auto_white_balance, 1)

#     # Wait for the auto exposure and white balance to stabilize
#     time.sleep(2)

#     try:
#         # Wait for a coherent color frame
#         frames = pipeline.wait_for_frames()
#         color_frame = frames.get_color_frame()
#         if not color_frame:
#             print("No color frame captured.")
#         else:
#             # Convert image to numpy array
#             color_image = np.asanyarray(color_frame.get_data())

#             if save:
#                 cv2.imwrite(path, color_image)

#             if visualize:
#                 # Display the image
#                 plt.imshow(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
#                 plt.axis('off')  # Turn off axis numbers and ticks
#                 plt.show()
#     finally:
#         # Stop streaming
#         pipeline.stop()

#     return color_image

# capture_img("04-07-24-testimage.png")
