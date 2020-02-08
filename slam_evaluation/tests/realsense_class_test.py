from __future__ import print_function
import os
import numpy as np
import time
from marker_detection.camera import Realsense

RS_POSE_FILE = "./slam_evaluation/data/rs_pose"

def test_rs(rs):
    rs_pose_file = open(RS_POSE_FILE + ".txt", "w")

    print("Recording data...")
    start_time = time.time()
    while True:

        pose = rs.read()
        quaternion = rs.get_quaternion()

        print(str.format("x = {0:.3f}, y = {1:.3f}, z = {2:.3f}",
                         pose.translation.x,
                         pose.translation.y,
                         pose.translation.z))

        timestamp = time.time() - start_time

        rs_pose_file.write(str(timestamp) + " " +
                           str(pose.translation.x) + " " +
                           str(pose.translation.y) + " " +
                           str(pose.translation.z) + " " +
                           str(quaternion[0]) + " " +
                           str(quaternion[1]) + " " +
                           str(quaternion[2]) + " " +
                           str(quaternion[3]) + "\n")
        time.sleep(0.1)

    rs.stop()

def main():
    # Connect to the realsense
    rs = Realsense()

    # Record GPS and realsense data
    test_rs(rs)

if __name__ == "__main__":
    main()
