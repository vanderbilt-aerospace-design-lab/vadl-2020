import os
import argparse
from marker_detection.marker_tracker import ArucoTracker, ColorMarkerTracker

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Tests for marker detection')
parser.add_argument('-v','--video', default=0,
                    help="Play video instead of live stream.")
parser.add_argument("-p", "--picamera", type=int, default=-1,
 	help="Indicates whether or not the Raspberry Pi camera should be used")
parser.add_argument('-d',"--debug", default=0,
                   help="Whether or not videos should be saved and print statements should be used.")
parser.add_argument('-r','--resolution', type=int, default=480,
                    help="Camera resolution")
parser.add_argument('-f','--fps', type=int, default=30,
                    help="Camera frame rate")
parser.add_argument('--dir', default=None,
                    help="Directory to save file")
parser.add_argument('-n','--name', default=None,
                    help="File name")

args = vars(parser.parse_args())

if not isinstance(args["video"], int):
    if not os.path.exists(args["video"]):
        raise Exception("ERROR: Video file does not exist")
    VIDEO_FILE_STREAM = args["video"]
else:
    VIDEO_FILE_STREAM = 0

# Pick resolution
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


if not isinstance(args["video"], int):
    if not os.path.exists(args["video"]):
        raise Exception("ERROR: Video file does not exist")
    VIDEO_FILE_STREAM = args["video"]
else:
    VIDEO_FILE_STREAM = 0

def main():
    print("Initializing")
    yellow_tracker = ColorMarkerTracker(src=args["video"],
                                        use_pi=args["picamera"],
                                        resolution=args["resolution"],
                                        framerate=args["fps"],
                                        debug=args["debug"],
                                        video_dir=args["dir"],
                                        video_file=args["name"])
    print("Tracking")
    while True:
        yellow_tracker.track_marker()
        if yellow_tracker.is_marker_found():
            print("found")
            print(yellow_tracker.get_pose())


    #aruco_tracker = ArucoTracker(src=args["video"],
    #                             use_pi=args["picamera"],
    #                             debug=args["debug"],
    #                             resolution=args["resolution"],
    #                             framerate=args["fps"],
    #                             video_dir=args["dir"],
    #                             video_file=args["name"])
    #print("Tracking")
    #while True:
    #    aruco_tracker.track_marker()
    #    if aruco_tracker.is_marker_found():
    #        print("found")
    #        print(aruco_tracker.get_pose())

if __name__ == "__main__":
    main()
