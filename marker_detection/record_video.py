import argparse
import os
from camera import VideoStreamer, VideoWriter

''' Record a video with a USB/webcam or the Picam.'''
VIDEO_DIR = "marker_detection/videos"

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Fly a UAV to a set altitude and hover over a marker')
parser.add_argument('-v','--video', default=0,
                    help="Play video instead of live stream.")
parser.add_argument("-p", "--picamera", type=int, default=-1,
 	help="Indicates whether or not the Raspberry Pi camera should be used")
parser.add_argument('-d','--dir', default=VIDEO_DIR,
                    help="Directory to save file in")
parser.add_argument('-n','--file', default=None,
                    help="File name to save video")

# TODO: Fix resolution and frame rate not actually changing anything
parser.add_argument('-r','--resolution', default=480,
                    help="Camera resolution")
parser.add_argument('-f','--fps', default=30,
                    help="Camera frame rate")

args = vars(parser.parse_args())

if not isinstance(args["video"], int):
    if not os.path.exists(args["video"]):
        raise Exception("ERROR: Video file does not exist")
    VIDEO_FILE_STREAM = args["video"]
else:
    VIDEO_FILE_STREAM = 0

# Currently only supports 1080p and 480p
# The Arducam is actually 1920 x 1088 and fails w/ 1080 set
if args["resolution"] == 1080:
    args["resolution"] = (1920, 1088)
else:
    args["resolution"] = (640, 480)


def main():
    vs = VideoStreamer(src=args["video"],
                       use_pi=args["picamera"],
                       resolution=args["resolution"],
                       framerate=args["fps"])
    vw = VideoWriter(video_dir=args["dir"],
                     video_file=args["file"],
                     resolution=args["resolution"],
                     framerate=args["fps"])

    print("Capturing images...")
    while True:
        vw.write(vs.read())


if __name__ == "__main__":
    main()

