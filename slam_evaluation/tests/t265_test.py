from __future__ import print_function
import os
import numpy as np
import time

# Set MAVLink protocol to 2.
os.environ["MAVLINK20"] = "1"

# Import the libraries
import pyrealsense2 as rs

RS_POSE_FILE = "./slam_evaluation/data/pose"
RS_ACCEL_FILE = "./slam_evaluation/data/rs_accel"


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

    # Forward facing
    H_T265body_aeroBody = np.array([[-1, 0, 0, 0],
                                    [0, 0, -1, 0],
                                    [0, -1, 0, 0],
                                    [0, 0, 0, 1]])

    # Original
    pose = np.array([data.translation.x, data.translation.y, data.translation.z, 1])

    # Forward facing / 45 degrees
    pose = np.matmul(pose, H_T265body_aeroBody)

    return pose

def test_rs(pipe):
    pose_file = open(RS_POSE_FILE + ".txt", "w")
    accel_file = open(RS_ACCEL_FILE + ".txt", "w")

    print("Recording data...")
    start_time = time.time()
    while True:

        # Wait for frames
        frames = pipe.wait_for_frames()

        # Get pose frame
        data = frames.get_pose_frame()
        if data:

            # Pose data consists of translation and rotation
            pose = data.get_pose_data()

            timestamp = time.time() - start_time

            pose_file.write(str(timestamp) + " " +
                               str(pose.translation.x) + " " +
                               str(pose.translation.y) + " " +
                               str(pose.translation.z) + " " +
                               str(pose.rotation.w) + " " +
                               str(pose.rotation.x) + " " +
                               str(pose.rotation.y) + " " +
                               str(pose.rotation.z) + "\n")

            accel_file.write(str(timestamp) + " " +
                            str(pose.acceleration.x) + " " +
                            str(pose.acceleration.y) + " " +
                            str(pose.acceleration.z) + "\n")



        # Sleep to preserve frequency
        time.sleep(.1)


def main():
    # Connect to the realsense
    pipe = realsense_connect()

    # Record GPS and realsense data
    test_rs(pipe)


if __name__ == "__main__":
    main()
