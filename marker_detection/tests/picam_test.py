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

FILE = args["dir"] + NAME

# Pick resolution
if args["resolution"] == 2464:
    args["resolution"] = (3280, 2464)
if args["resolution"] == 1080:
    args["resolution"] = (1920, 1080)
elif args["resolution"] == 720:
    args["resolution"] = (1280, 720)
elif args["resolution"] == 480:
    args["resolution"] = (640, 480)
elif args["resolution"] == 240:
    args["resolution"] = (352, 240)
elif args["resolution"] == 144:
    args["resolution"] = (256, 144)
else:
    args["resolution"] = (64, 64)

# Initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = args["resolution"]
camera.start_preview()

# Allow the camera to warm up
time.sleep(2)

# Capture image and save to file
camera.capture(FILE)