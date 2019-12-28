from __future__ import print_function
from imutils.video import FPS
import argparse
import cv2

''' This script will quantify the FPS of whatever function you want. Just place it in the while loop. Input as a 
    command line arg how many frames you would like to process'''

# construct the argument parse and parse the arguments
parser = argparse.ArgumentParser()
parser.add_argument("-n", "--num-frames", type=int, default=100,
	help="# of frames to loop over for FPS test")
parser.add_argument("-p", "--picamera", type=int, default=-1,
 	help="Indicates whether or not the Raspberry Pi camera should be used")
args = vars(parser.parse_args())

print("Starting FPS counter...")
fps = FPS().start()

# loop over some frames
while fps._numFrames < args["num_frames"]:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 400 pixels

    ''' Place section under analysis here'''

    # update the FPS counter
    fps.update()

# stop the timer and display FPS information
fps.stop()
print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
