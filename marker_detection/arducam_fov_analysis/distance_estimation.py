
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

DEG_PER_PIX = 160.0 / 640.0
PIX_PER_DEG = 1.0 / DEG_PER_PIX


# Function to compute ground point in an image given roll and pitch of camera
# - ppd: pixel per degree ratio for the fish-eye lens
# - returns: (x,y) offset of the ground point in pixels from the center of the image
def calc_gnd_pt_offset(roll, pitch, ppd=PIX_PER_DEG):

    # Find center of hemisphere in pixels
    diam = ppd * 180.0

    # Precompute some values
    t = diam / 2.0
    t_sq = t**2
    sin_2_p = sin(radians(pitch)) ** 2.0
    sin_2_r = sin(radians(roll)) ** 2.0

    # Compute x^2, x
    x_sq = (t_sq * sin_2_r * (1 - sin_2_p)) / (1 - sin_2_r * sin_2_p + 0.00000001)
    x = sqrt(x_sq)
    vw = VideoWriter(video_dir="/home/pi/pi_videos/",
                     video_file="drone_pi_" + str(int(time.time())),
                     resolution=480, framerate=20)
    # Compute y
    y = sqrt(t_sq * sin_2_p * (1 - x_sq / t_sq))

    return [x, y]


# Function to mark an image at a specific point
def mark_point(img, x, y, color, size):
    cv2.circle(img, (int(x), int(y)), size, color, thickness=-1)
    return img


# Function to log attitude data alongside video frames
def log_attitude_and_video(log_dir, vs, duration=30):

    # Connect to dronekit
    vehicle = connect_vehicle()

    # Open new log file
    new_dir_name = log_dir + "/log_" + str(int(time.time()))
    os.mkdir(new_dir_name)
    att_log = open(new_dir_name + "/" + "attitude_log.txt", 'w+')

    # Create a VideoWriter object
    vw = VideoWriter(video_dir=new_dir_name,
                     video_file="video_log",
                     resolution=480, framerate=20)


    # Print start message
    print("Recording attitude and video data for " + str(duration) + " seconds")

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


# Function to add a marker indicating the ground point position to a video
def overlay_gnd_pt(data_dir, video_file, attitude_file, new_video_file):

    # Create VideoStreamer to read file
    fs = VideoStreamer(src=data_dir + "/" + video_file)

    # Create VideoWriter for modified video
    vw = VideoWriter(video_dir=data_dir, video_file=new_video_file, framerate=20)

    # Open attitude file
    att_data = open(data_dir + "/" + attitude_file, "r")

    # Construct the overlayed video
    for line in att_data:

        # Get the next data pair
        rpy = [float(i) for i in line.split(",")]
        frame = fs.read()

        # Overlay the ground point
        gp_offset = calc_gnd_pt_offset(degrees(rpy[0]), degrees(rpy[1]))
        gp = [gp_offset[0] + 640 / 2.0, -gp_offset[1] + 480.0 / 2.0]
        modified_frame = mark_point(frame, gp[0], gp[1], color=(255,0,0), size=5)

        # Write modified frame to new video
        vw.write(modified_frame)

    # Close video writer
    vw.stop()


if __name__ == "__main__":

    # Create video stream
    vs = VideoStreamer(use_pi=True, framerate=20)

    # Log data
    log_attitude_and_video("/home/pi/attitude_logs", vs, duration=10)

    # # Overlay ground point on video
    # overlay_gnd_pt(data_dir="/home/jake/Desktop/dump/attitude_logs/log_1582267057",
    #                video_file="video_log.avi",
    #                attitude_file="attitude_log.txt",
    #                new_video_file="overlayed.avi")





