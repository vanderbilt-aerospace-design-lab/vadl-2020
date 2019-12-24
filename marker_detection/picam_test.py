from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time

IMAGE_DIR = "marker_detection/images"

# Initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (1920, 1080)
raw_capture = PiRGBArray(camera, size=(1920, 1080))

# Allow the camera to warm up
time.sleep(0.1)

ct = 0
# Read images from camera
for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    img = frame.array

    # Clear the stream in preparation for the next frame
    raw_capture.truncate(0)

    # Save an image every X time interval
    cv2.imwrite(IMAGE_DIR + '/marker_test_{}.jpg'.format(ct), img)
    print("Saved image {}".format(ct))
    ct += 1
    time.sleep(1)
    if ct > 0:
        break
