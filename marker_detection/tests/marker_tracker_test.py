import os
import argparse
from marker_detection.marker_tracker import ArucoTracker, ColorMarkerTracker
#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Fly a UAV to a set altitude and hover over a marker.')
parser.add_argument('-v','--video', default=0,
                    help="Play video instead of live stream.")
parser.add_argument("-p", "--picamera", type=int, default=-1,
 	help="Indicates whether or not the Raspberry Pi camera should be used")
parser.add_argument('-d',"--debug", default=0,
                   help="Vehicle connection target string. If specified, SITL will be used.")

args = vars(parser.parse_args())

if not isinstance(args["video"], int):
    if not os.path.exists(args["video"]):
        raise Exception("ERROR: Video file does not exist")
    VIDEO_FILE_STREAM = args["video"]
else:
    VIDEO_FILE_STREAM = 0

def main():
    mt = ArucoTracker(src=args["video"], use_pi=args["picamera"], debug=args["debug"], resolution=(1920, 1080), framerate=20)

    while True:
        mt.track_marker()
        if mt.is_marker_found():
            print(mt.get_pose())

if __name__ == "__main__":
    main()