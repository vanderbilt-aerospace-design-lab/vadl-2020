from marker_detection.camera import VideoStreamer, VideoWriter
import argparse
import os
import time
import cv2
import numpy as np

VIDEO_DIR = "marker_detection/videos"

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Test the VideoStreamer and VideoWriter classes')
parser.add_argument('-v','--video', default=0,
                    help="Play video instead of live stream.")
parser.add_argument("-p", "--picamera", type=int, default=-1,
 	help="Indicates whether or not the Raspberry Pi camera should be used")
parser.add_argument('-r','--resolution', type=int, default=480,
                    help="Camera resolution")
parser.add_argument('-f','--fps', type=int, default=30,
                    help="Camera frame rate")
parser.add_argument('-d','--dir', default=VIDEO_DIR,
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

def main():
    vs = VideoStreamer(src=args["video"],
                       use_pi=args["picamera"],
                       resolution=args["resolution"],
                       framerate=args["fps"])

    ''' Test VideoStreamer'''
    print("Capturing images...")
    for i in range(0, 50):
        frame = vs.read()
        if args["picamera"] < 1:
            cv2.imshow("Image", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
        print(np.shape(frame))

    # ''' Test VideoWriter'''
    # vw = VideoWriter(video_dir=args["dir"],
    #                  video_file=args["name"],
    #                  resolution=args["resolution"],
    #                  framerate=args["fps"])
    # start = time.time()
    # while (time.time() - start) < 3:
    #     vw.write(vs.read())

    vs.stop()
    # vw.release()

if __name__ == "__main__":
    main()
