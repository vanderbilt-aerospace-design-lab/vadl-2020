from __future__ import print_function
import os
import argparse
import numpy as np
from dronekit import VehicleMode
import time
import transformations as tf
from utils import dronekit_utils, file_utils

# Set MAVLink protocol to 2.
os.environ["MAVLINK20"] = "1"

# Import the libraries
import pyrealsense2 as rs

DATA_DIR = "./slam_evaluation/data"
GPS_FILE_BASE = "gps_pose"
RS_FILE_BASE = "rs_pose"
GPS_POSE_FILE = file_utils.create_file_name_chronological(DATA_DIR,GPS_FILE_BASE,"txt")
RS_POSE_FILE = file_utils.create_file_name_chronological(DATA_DIR, RS_FILE_BASE, "txt")

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

def ned_to_body(location_ned, attitude):
    roll_R = tf.rotation_matrix(attitude.roll, (1, 0, 0))[0:3, 0:3]
    pitch_R = tf.rotation_matrix(attitude.pitch, (0, 1, 0))[0:3, 0:3]
    yaw_R = tf.rotation_matrix(attitude.yaw, (0, 0, 1))[0:3, 0:3]
    transform = np.matmul(np.matmul(roll_R, pitch_R), yaw_R)

    ned_mat = np.array([location_ned.north, location_ned.east, location_ned.down])

    return np.matmul(ned_mat, transform)

def rs_to_body(data):

    # Forward facing
    H_aeroRef_T265Ref = np.array([[0, 0, -1, 0], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]])
    H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

    # Original
    pose = np.array([data.translation.x, data.translation.y, data.translation.z, 1])

    # Forward facing / 45 degrees
    pose = np.matmul(pose, H_T265body_aeroBody)

    return pose

def record_data(vehicle, pipe):
    ct = 0

    # pose files to save to
    # gps_pose_file = open(GPS_POSE_FILE, "w")
    # rs_pose_file = open(RS_POSE_FILE, "w")

    gps_pose_file = file_utils.open_file(DATA_DIR, GPS_POSE_FILE)
    rs_pose_file = file_utils.open_file(DATA_DIR, RS_POSE_FILE)

    ''' Only for testing'''
    # dronekit_utils.arm(vehicle)

    while not vehicle.armed:
        print("Waiting for arming")
        time.sleep(1)

    ''' Only for testing'''
    # dronekit_utils.takeoff(vehicle, 2)

    print("Recording data...")
    start_time = time.time()
    while vehicle.armed:
        frame_time = time.time()

        # Wait for frames
        frames = pipe.wait_for_frames()

        # Get pose frame
        data = frames.get_pose_frame()
        if data:

            # Pose data consists of translation and rotation
            rs_pose = data.get_pose_data()

            # Transform RS frame to body frame
            rs_pose_body_frame = rs_to_body(rs_pose)

            # Retrieve UAV location in local NED frame
            gps_pose_ned = vehicle.location.local_frame

            # Get UAV attitude
            attitude = vehicle.attitude

            # Transform NED frame to body frame using roll, pitch, yaw
            gps_pose_body = ned_to_body(gps_pose_ned, attitude)

            # Convert attitude to a quaternion
            uav_quat = tf.quaternion_from_euler(attitude.roll, attitude.pitch, attitude.yaw)

            # Record time
            timestamp = time.time() - start_time

            # Write to files
            gps_pose_file.write(str(timestamp) + " " +
                                str(gps_pose_body[0]) + " " +
                                str(gps_pose_body[1]) + " " +
                                str(gps_pose_body[2]) + " " +
                                str(uav_quat[0]) + " " +
                                str(uav_quat[1]) + " " +
                                str(uav_quat[2]) + " " +
                                str(uav_quat[3]) + "\n")
            rs_pose_file.write(str(timestamp) + " " +
                               str(rs_pose_body_frame[0]) + " " +
                               str(rs_pose_body_frame[1]) + " " +
                               str(rs_pose_body_frame[2]) + " " +
                               str(rs_pose.rotation.w) + " " +
                               str(rs_pose.rotation.x) + " " +
                               str(rs_pose.rotation.y) + " " +
                               str(rs_pose.rotation.z) + "\n")

        # Sleep to preserve frequency
        time.sleep((1.0 / FREQ) - time.time() + frame_time)

        '''Testing'''
        # if ct == 0:
        #     dronekit_utils.goto_position_target_body_offset_ned(vehicle, forward=2, right=0, down=0)
        #     time.sleep(10)
        # if ct==1:
        #     dronekit_utils.goto_position_target_body_offset_ned(vehicle, forward=0, right=2, down=0)
        #     time.sleep(10)
        # if ct==2:
        #     dronekit_utils.goto_position_target_body_offset_ned(vehicle, forward=0, right=0, down=2)
        #     time.sleep(10)
        # ct +=1


def main():
    # Connect to the Pixhawk
    vehicle = connect_vehicle()

    # Connect to the realsense
    pipe = realsense_connect()

    # wait for GPS to initialize home location
    dronekit_utils.wait_for_home_location(vehicle)

    # Record GPS and realsense data
    record_data(vehicle, pipe)


if __name__ == "__main__":
    main()
