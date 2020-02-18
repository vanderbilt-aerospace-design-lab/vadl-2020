
from marker_detection.camera import *
import time

# TODO Add cmdline args for filename, resolution, framerate, and record time

if __name__ == "__main__":

    # Create VideoStreamer and VideoWriter objects
    vs = VideoStreamer(use_pi=1, resolution=480, framerate=25)
    vw = VideoWriter(video_dir="/home/pi/pi_videos/",
                     video_file="drone_pi_" + str(time.time()),
                     resolution=480, framerate=25)

    # Record indefinitely
    while True:
        vw.write(vs.read())


