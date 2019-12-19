from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time

''' Run from root directory '''
CAMERA_VIDEO_FILE = "flight_videos/flight_0.avi"
FRAME_RATE = 60
RESOLUTION = (1920, 1080)

# Initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (1920,1080)
raw_capture = PiRGBArray(camera, size=(1920,1080))

# Allow the camera to warm up
time.sleep(0.1)

fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(CAMERA_VIDEO_FILE, fourcc, FRAME_RATE, RESOLUTION)

# Read images from camera
for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    img = frame.array

    # Clear the stream in preparation for the next frame
    raw_capture.truncate(0)

    out.write(img)

out.release()


