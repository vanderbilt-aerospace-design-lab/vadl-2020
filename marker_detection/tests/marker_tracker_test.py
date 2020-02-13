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
                    help="Directory to save file. Defaults to marker_detection/videos")
parser.add_argument('-n','--name', default=None,
                    help="File name. If none specified, defaults to the current date.")
parser.add_argument('--pose_file', default=None,
                    help="Pose file name for debugging. Do not input directory, "
                         "just the file name w/ or w/o .txt at the end. If none specified, defaults to the current date.")

args = vars(parser.parse_args())

if not isinstance(args["video"], int):
    if not os.path.exists(args["video"]):
        raise Exception("ERROR: Video file does not exist")
    VIDEO_FILE_STREAM = args["video"]
else:
    VIDEO_FILE_STREAM = 0

if not isinstance(args["video"], int):
    if not os.path.exists(args["video"]):
        raise Exception("ERROR: Video file does not exist")
    VIDEO_FILE_STREAM = args["video"]
else:
    VIDEO_FILE_STREAM = 0

def main():
    print("Initializing")
#    tracker = ColorMarkerTracker(src=args["video"],
 #                                       use_pi=args["picamera"],
  #                                      resolution=args["resolution"],
   #                                     framerate=args["fps"],
    #                                    debug=args["debug"],
     #                                   video_dir=args["dir"],
      #                                  video_file=args["name"])
    tracker = ArucoTracker(src=args["video"],
                                 use_pi=args["picamera"],
                                 debug=args["debug"],
                                 resolution=args["resolution"],
                                 framerate=args["fps"],
                                 video_dir=args["dir"],
                                 video_file=args["name"],
                                 pose_file=args["pose_file"])
    print("Tracking")
    while True:
        tracker.track_marker()
        if tracker.is_marker_found():
            print(tracker.get_pose())


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
