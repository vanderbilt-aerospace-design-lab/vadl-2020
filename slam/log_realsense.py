### ------------------------------------------------------------ ###
#
# This script logs pose and vibration data from the Realsense.
# It is intended for use during manual flights so you can evaluate the Realsense data
# afterwards. It is recommended to log data with this script and make sure tracking is stable
# before attempting to actually fly the UAV in SLAM mode. And by "recommended" I mean do this unless
# you'd like to see your UAV decide to spontaneously race to the moon.

# Import the libraries
from __future__ import print_function
import os
import time
import pyrealsense2 as rs

# Custom packages
from marker_detection.camera import Realsense
from utils import file_utils

# Set MAVLink protocol to 2.
os.environ["MAVLINK20"] = "1"

## Parameters ##
DATA_DIR = "slam/data" # Save pose and vibration data
RS_FILE_BASE = "rs_pose"
ACCEL_FILE_BASE = "rs_accel"
RS_POSE_FILE = file_utils.create_file_name_chronological(DATA_DIR, RS_FILE_BASE, "txt")
RS_ACCEL_FILE = file_utils.create_file_name_chronological(DATA_DIR, ACCEL_FILE_BASE, "txt")
# Open the files to save data to
rs_pose_file = file_utils.open_file(RS_POSE_FILE)
rs_accel_file = file_utils.open_file(RS_ACCEL_FILE)


# Receive pose and vibration data from the Realsense and save to files.
# hese logs follow the .tum format so they can be processed by
# the EVO SLAM comparisons package:
#
# https://github.com/MichaelGrupp/evo
def log_realsense(rs):

    print("Recording data...")
    start_time = time.time()

    while True:
        
        # Read data from the Realsense
        pose = rs.read()

        # Record the time
        timestamp = time.time() - start_time

        # Save pose and vibration data to disk
        rs_pose_file.write(str(timestamp) + " " +
                       str(pose.translation.x) + " " +
                       str(pose.translation.y) + " " +
                       str(pose.translation.z) + " " +
                       str(pose.rotation.w) + " " +
                       str(pose.rotation.x) + " " +
                       str(pose.rotation.y) + " " +
                       str(pose.rotation.z) + "\n")

        rs_accel_file.write(str(timestamp) + " " +
                        str(pose.acceleration.x) + " " +
                        str(pose.acceleration.y) + " " +
                        str(pose.acceleration.z) + "\n")

        # Sleep to preserve a frequency
        time.sleep(.1)


def main():

    # Connect to the realsense
    rs = Realsense()

    # Record GPS and realsense data
    log_realsense(rs)


if __name__ == "__main__":
    main()
