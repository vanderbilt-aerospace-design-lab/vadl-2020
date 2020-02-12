from __future__ import print_function
from imutils.video import FPS
import argparse
import cv2
from marker_detection.marker_tracker import ColorMarkerTracker, ArucoTracker

''' This script will quantify the FPS of whatever function you want. Just place it in the while loop. Input as a 
    command line arg how many frames you would like to process'''

DEFAULT_FREQ = 30 # Hz
DEFAULT_MARKER = "aruco"

# construct the argument parse and parse the arguments
parser = argparse.ArgumentParser()
parser.add_argument("-n", "--num-frames", type=int, default=100,
	help="# of frames to loop over for FPS test")
#Set up option parsing to get connection string
parser.add_argument('-v','--video', default=0,
                    help="Play video instead of live stream.")
parser.add_argument("-p", "--pi", type=int, default=1,
 	                help="Indicates whether or not the Raspberry Pi camera should be used. Defaults to Pi.")
parser.add_argument('-d',"--debug", default=0,
                    help="Whether or not videos and pose files should be saved and print statements are used.")
parser.add_argument('-r','--resolution', type=int, default=480,
                    help="Camera resolution")
parser.add_argument('-f','--fps', type=int, default=DEFAULT_FREQ,
                    help="Camera frame rate")
parser.add_argument('--dir', default=None,
                    help="Directory to save file. Defaults to marker_detection/videos")
parser.add_argument('--pose_file', default=None,
                    help="Pose file name for debugging. Do not input directory, "
                         "include .txt at the end. If no file specified, defaults to the current date.")
parser.add_argument('-m', '--marker', default=DEFAULT_MARKER,
                    help="Type of marker to track. 'aruco' or 'yellow'")
parser.add_argument('--frequency', default=DEFAULT_FREQ,
                    help="Frequency of marker tracking")

args = vars(parser.parse_args())

marker_tracker = ColorMarkerTracker(src=args["video"],
                                    use_pi=args["pi"],
                                    resolution=args["resolution"],
                                    framerate=args["fps"],
                                    freq=args["frequency"],
                                    debug=args["debug"])

print("Starting FPS counter...")
fps = FPS().start()

# loop over some frames
while fps._numFrames < args["num_frames"]:
    ''' Place section under analysis here'''

    marker_tracker.track_marker()

    # update the FPS counter
    fps.update()

# stop the timer and display FPS information
fps.stop()
print("[INFO] elapsed time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
