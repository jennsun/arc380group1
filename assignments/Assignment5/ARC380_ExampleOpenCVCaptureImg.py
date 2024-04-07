import cv2
import time

# Initialize the video capture object with the camera index
cap = cv2.VideoCapture(2)  # Adjust the index if necessary

if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Set the desired resolution
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

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
    cv2.imwrite('captured_image.jpg', frame)
else:
    print("Failed to capture frame")

# Release the video capture object
cap.release()