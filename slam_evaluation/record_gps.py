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

GPS_POSE_FILE = "./slam_evaluation/data/gps_pose"
RS_POSE_FILE = "./slam_evaluation/data/rs_pose"
FREQ = 30 # Hz

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Record GPS and Realsense data.')
parser.add_argument('--sitl',
                   help="Vehicle connection target string. If specified, SITL will be used.")

args = vars(parser.parse_args())


def connect_vehicle():
    # Start SITL if connection string specified
    if args["sitl"]:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        CONNECTION_STRING = sitl.connection_string()
    else:
        CONNECTION_STRING = "/dev/ttyAMA0"

    # Connect to the Vehicle
    return dronekit_utils.connect_vehicle(CONNECTION_STRING)

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

def wait_for_home_location(vehicle):
    # Wait for GPS to set home location
    while not vehicle.home_location:
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        print("Waiting for home location")
        time.sleep(0.5)

# TODO: Determine if the RPY of the RS ref. frame have to be rotated at all to be consistent with the UAV Compass RPY.
# TODO: Make a separate script that calculates the rel. pose of the GPS location
# TODO: Test this on the UAV
def record_data(vehicle, pipe):

    # Set up RS stream and wait for frames
    frames = pipe.wait_for_frames()

    # pose files to save to
    gps_pose_file = open(GPS_POSE_FILE + ".txt", "w")
    rs_pose_file = open(RS_POSE_FILE + ".txt", "w")

    while not vehicle.armed:
        print("Waiting for arming")
        time.sleep(1)
        vehicle.armed = True

    print("Recording data...")
    start_time = time.time()
    while vehicle.armed:
        frame_time = time.time()
        # Get pose frame
        data = frames.get_pose_frame()

        if data:

            # Pose data consists of translation and rotation
            pose = data.get_pose_data()

            # Retrieve global GPS coordinates w/ relative alt from vehicle
            # Subtract the home location to obtain full relative coordinates
            location_gps = vehicle.location.global_relative_frame
            gps_pose_x = location_gps.lat - vehicle.home_location.lat
            gps_pose_y = location_gps.lon - vehicle.home_location.lon
            gps_pose_z = location_gps.alt

            # Get UAV attitude
            attitude = vehicle.attitude
            uav_quat = tf.quaternion_from_euler(attitude.roll, attitude.pitch, attitude.yaw)

            timestamp = time.time() - start_time

            # Write to files
            gps_pose_file.write(str(timestamp) + " " +
                                str(gps_pose_x) + " " +
                                str(gps_pose_y) + " " +
                                str(gps_pose_z) + " " +
                                str(uav_quat[0]) + " " +
                                str(uav_quat[1]) + " " +
                                str(uav_quat[2]) + " " +
                                str(uav_quat[3]) + "\n")
            rs_pose_file.write(str(timestamp) + " " +
                               str(pose.translation.x) + " " +
                               str(pose.translation.y) + " " +
                               str(pose.translation.z) + " " +
                               str(pose.rotation.w) + " " +
                               str(pose.rotation.x) + " " +
                               str(pose.rotation.y) + " " +
                               str(pose.rotation.z) + "\n")

        time.sleep((1.0 / FREQ) - time.time() + frame_time)


def main():
    # Connect to the Pixhawk
    vehicle = connect_vehicle()

    # Connect to the realsense
    pipe = realsense_connect()

    # wait for GPS to initialize home location
    wait_for_home_location(vehicle)

    # Record GPS and realsense data
    record_data(vehicle, pipe)


if __name__ == "__main__":
    main()
