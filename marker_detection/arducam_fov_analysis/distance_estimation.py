
# --------------------------------------------------------------------------------- #
#
# This script is for testing distance estimation using arducam based on the
# analysis done in arducam_fov_analysis.py. Assuming the camera is at a 90 degree
# angle to a flat surface, the distance along the ground to any pixel in the image
# can be easily calcutated. This script displays an image and outputs the estimated
# distance to any pixel in the image after a mouse click on the image
#
# --------------------------------------------------------------------------------- #

from math import *
import cv2
import time
import os
from marker_detection.camera import *
from utils.dronekit_utils import *

DEG_PER_PIX = 160 / 640
PIX_PER_DEG = 1 / DEG_PER_PIX


# Function to compute ground point in an image given roll and pitch of camera
# - ppd: pixel per degree ratio for the fish-eye lens
# - returns: (x,y) offset of the ground point in pixels from the center of the image
def calc_gnd_pt_offset(roll, pitch, ppd=PIX_PER_DEG):

    # Find center of hemisphere in pixels
    diam = ppd * 180
    center = [diam / 2, diam / 2]

    # Precompute some values
    t = diam / 2
    t_sq = t**2
    sin_2_p = sin(radians(pitch)) ** 2
    sin_2_r = sin(radians(roll)) ** 2

    # Compute x^2, x
    x_sq = (t_sq * sin_2_r * (1 - sin_2_p)) / (1 - sin_2_r * sin_2_p + 0.00000001)
    x = sqrt(x_sq)
    vw = VideoWriter(video_dir="/home/pi/pi_videos/",
                     video_file="drone_pi_" + str(int(time.time())),
                     resolution=480, framerate=25)
    # Compute y
    y = sqrt(t_sq * sin_2_p * (1 - x_sq / t_sq))

    return [center[0] + x, center[1] + y]


# Function to mark an image at a specific point
def mark_point(img, x, y, color, size):
    pass


# Function to log attitude data alongside video frames
def log_attitude_and_video(log_dir, vs, duration=30):

    # Connect to dronekit
    vehicle = connect_vehicle()

    # Open new log file
    new_dir_name = log_dir + "/log_" + str(int(time.time()))
    os.mkdir(new_dir_name)
    att_log = open(new_dir_name + "/" + "attitude_log.txt")

    # Create a VideoWriter object
    vw = VideoWriter(video_dir=new_dir_name,
                     video_file="video_log",
                     resolution=480, framerate=10)

    # Record for specified duration
    start = time.time()

    while time.time() - start < duration:

        # Log attitude data from vehicle
        att_log.write(str(vehicle.attitude.roll) + ",")
        att_log.write(str(vehicle.attitude.pitch) + ",")
        att_log.write(str(vehicle.attitude.yaw) + "\n")

        # Write new video frame
        vw.write(vs.read())

    vw.stop()


if __name__ == "__main__":

    # Create video stream
    vs = VideoStreamer(use_pi=True, framerate=10)

    # Log data
    log_attitude_and_video("/home/pi/attitude_logs", vs, duration=10)





