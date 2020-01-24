from __future__ import print_function
import os
import numpy as np
import time
from marker_detection.camera import Realsense

RS_POSE_FILE = "./slam_evaluation/data/rs_pose"

def rs_to_body(rs_pose):

    # Forward facing
    H_T265body_aeroBody = np.array([[-1, 0, 0, 0],
                                    [0, 0, -1, 0],
                                    [0, -1, 0, 0],
                                    [0, 0, 0, 1]])

    # Original
    pose = np.array([rs_pose[0], rs_pose[1], rs_pose[2], 1])

    # Forward facing / 45 degrees
    pose = np.matmul(pose, H_T265body_aeroBody)

    return pose

def test_rs(rs):
    rs_pose_file = open(RS_POSE_FILE + ".txt", "w")

    print("Recording data...")
    start_time = time.time()
    while True:

        pose = rs.read()
        quaternion = rs.get_quaternion()

        # Transform RS frame to body frame
        rs_pose_body_frame = rs_to_body(pose)

        print(str.format("x = {0:.3f}, y = {1:.3f}, z = {2:.3f}",
                         rs_pose_body_frame[0],
                         rs_pose_body_frame[1],
                         rs_pose_body_frame[2]))

        timestamp = time.time() - start_time

        rs_pose_file.write(str(timestamp) + " " +
                           str(rs_pose_body_frame[0]) + " " +
                           str(rs_pose_body_frame[1]) + " " +
                           str(rs_pose_body_frame[2]) + " " +
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
