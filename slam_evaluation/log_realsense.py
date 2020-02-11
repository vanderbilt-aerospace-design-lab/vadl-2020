from __future__ import print_function
import os
import time
from marker_detection.camera import Realsense
from utils import file_utils

# Set MAVLink protocol to 2.
os.environ["MAVLINK20"] = "1"

# Import the libraries
import pyrealsense2 as rs
DATA_DIR = "/home/vadl/catkin_ws/src/vadl-2020/slam_evaluation/data"
RS_FILE_BASE = "rs_pose"
ACCEL_FILE_BASE = "rs_accel"
RS_POSE_FILE = file_utils.create_file_name_chronological(DATA_DIR, RS_FILE_BASE, "txt")
RS_ACCEL_FILE = file_utils.create_file_name_chronological(DATA_DIR, ACCEL_FILE_BASE, "txt")
# pose files to save to
rs_pose_file = file_utils.open_file(RS_POSE_FILE)
rs_accel_file = file_utils.open_file(RS_ACCEL_FILE)

def log_realsense(rs):

    print("Recording data...")
    start_time = time.time()
    while True:

        pose = rs.read()

        timestamp = time.time() - start_time

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

        # Sleep to preserve frequency
        time.sleep(.1)


def main():
    # Connect to the realsense
    rs = Realsense()

    # Record GPS and realsense data
    log_realsense(rs)


if __name__ == "__main__":
    main()
