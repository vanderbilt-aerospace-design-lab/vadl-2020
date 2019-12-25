from marker_detection.camera import VideoStreamer
import argparse
import os
import cv2
import numpy as np

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Fly a UAV to a set altitude and hover over a marker')
parser.add_argument('-v','--video', default=0,
                    help="Play video instead of live stream.")
parser.add_argument("-p", "--picamera", type=int, default=-1,
 	help="Indicates whether or not the Raspberry Pi camera should be used")
parser.add_argument('-r','--resolution', type=int, default=480,
                    help="Camera resolution")
parser.add_argument('-f','--fps', type=int, default=30,
                    help="Camera frame rate")

args = vars(parser.parse_args())

if not isinstance(args["video"], int):
    if not os.path.exists(args["video"]):
        raise Exception("ERROR: Video file does not exist")
    VIDEO_FILE_STREAM = args["video"]
else:
    VIDEO_FILE_STREAM = 0

# Currently only supports 1080p and 480p
# The Arducam is actually 1920 x 1088 and will round 1080 up
if args["resolution"] == 1080:
    args["resolution"] = (1920, 1080)
elif args["resolution"] == 480:
    args["resolution"] = (640, 480)
else:
    args["resolution"] = (64, 64)

def main():
    vs = VideoStreamer(src=args["video"],
                       use_pi=args["picamera"],
                       resolution=args["resolution"],
                       framerate=args["fps"])

    print("Capturing images...")
    while True:
        frame = vs.read()
        if args["picamera"] < 1:
            cv2.imshow("Image", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
        print(np.shape(frame))


if __name__ == "__main__":
    main()
