from __future__ import print_function
import os
import argparse
import numpy as np
from dronekit import VehicleMode
import time
import transformations as tf
from utils import dronekit_utils

# Set MAVLink protocol to 2.
os.environ["MAVLINK20"] = "1"

# Import the libraries
import pyrealsense2 as rs

RS_POSE_FILE = "./slam_evaluation/data/rs_pose"


def realsense_connect():
    print("Connecting to Realsense")

    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs.pipeline()

    # Build config object before requesting data
    cfg = rs.config()

    # Enable the stream we are interested in
    cfg.enable_stream(rs.stream.pose)  # Positional data

    # Start streaming with requested config
    pipe.start(cfg)

    return pipe

def rs_to_body(data):

    # Downward facing
    H_T265body_aeroBody = np.array([[0,1,0,0],[1,0,0,0],[0,0,-1,0],[0,0,0,1]])

    # Forward facing
    #  H_aeroRef_T265Ref = np.array([[0, 0, -1, 0], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
    #  H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

    rotation_euler = tf.euler_from_quaternion([data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z])
    rot_mat_x = tf.rotation_matrix(rotation_euler[0], (1, 0, 0))
    rot_mat_y = tf.rotation_matrix(rotation_euler[1], (0, 1, 0))
    rot_mat_z = tf.rotation_matrix(rotation_euler[2], (0, 0, 1))
    transform = np.matmul(np.matmul(rot_mat_x, rot_mat_y), rot_mat_z)

    pose = np.array([data.translation.x, data.translation.y, data.translation.z, 1])
    return np.matmul(np.matmul(pose, transform), H_T265body_aeroBody)


def test_rs(pipe):
    rs_pose_file = open(RS_POSE_FILE + ".txt", "w")


    print("Recording data...")
    start_time = time.time()
    while True:

        # Wait for frames
        frames = pipe.wait_for_frames()

        # Get pose frame
        data = frames.get_pose_frame()
        if data:

            # Pose data consists of translation and rotation
            rs_pose = data.get_pose_data()

            # Transform RS frame to body frame
            rs_pose_body_frame = rs_to_body(rs_pose)

            print(rs_pose_body_frame)

            timestamp = time.time() - start_time

            rs_pose_file.write(str(timestamp) + " " +
                               str(rs_pose_body_frame[0]) + " " +
                               str(rs_pose_body_frame[1]) + " " +
                               str(rs_pose_body_frame[2]) + " " +
                               str(rs_pose.rotation.w) + " " +
                               str(rs_pose.rotation.x) + " " +
                               str(rs_pose.rotation.y) + " " +
                               str(rs_pose.rotation.z) + "\n")



        # Sleep to preserve frequency
        time.sleep(.1)


def main():
    # Connect to the realsense
    pipe = realsense_connect()

    # Record GPS and realsense data
    test_rs(pipe)


if __name__ == "__main__":
    main()
