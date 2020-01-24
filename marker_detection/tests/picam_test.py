from picamera import PiCamera
import time
import argparse

IMAGE_DIR = "marker_detection/images"

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Test the VideoStreamer and VideoWriter classes')
parser.add_argument('-r','--resolution', type=int, default=480,
                    help="Camera resolution")
parser.add_argument('-d','--dir', default=IMAGE_DIR,
                    help="Directory to save file")
parser.add_argument('-n','--name', default=None,
                    help="File name")

args = vars(parser.parse_args())

NAME = args["name"]
if args["name"] is None:
    NAME = "image.jpg"

FILE = args["dir"] + "/" + NAME

# Initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = args["resolution"]
camera.start_preview()

# Allow the camera to warm up
time.sleep(2)

# Capture image and save to file
camera.capture(FILE)