from __future__ import print_function
import os
import argparse
import numpy as np
from dronekit import VehicleMode
import time
import transformations as tf
from utils import dronekit_utils, file_utils
from marker_detection.camera import Realsense

# Set MAVLink protocol to 2.
os.environ["MAVLINK20"] = "1"

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

def ned_to_body(location_ned, attitude):
    roll_R = tf.rotation_matrix(attitude.roll, (1, 0, 0))[0:3, 0:3]
    pitch_R = tf.rotation_matrix(attitude.pitch, (0, 1, 0))[0:3, 0:3]
    yaw_R = tf.rotation_matrix(attitude.yaw, (0, 0, 1))[0:3, 0:3]
    transform = np.matmul(np.matmul(roll_R, pitch_R), yaw_R)

    ned_mat = np.array([location_ned.north, location_ned.east, location_ned.down])

    return np.matmul(ned_mat, transform)

def rs_to_body(rs_pose):

    # Forward facing
    H_T265body_aeroBody = np.array([[-1, 0, 0, 0],
                                    [0, 0, -1, 0],
                                    [0, -1, 0, 0],
                                    [0, 0, 0, 1]])

    # Original
    rs_pose = np.array([rs_pose[0], rs_pose[1], rs_pose[2], 1])

    # Forward facing / 45 degrees
    body_pose = np.matmul(rs_pose, H_T265body_aeroBody)

    return body_pose

def record_data(vehicle, rs):
    ct = 0

    # pose files to save to
    gps_pose_file = file_utils.open_file(GPS_POSE_FILE)
    rs_pose_file = file_utils.open_file(RS_POSE_FILE)

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

        # Get Realsense pose
        rs_pose = rs.read()

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
    vehicle = dronekit_utils.connect_vehicle_args(args)

    # Connect to the realsense
    rs = Realsense()

    # wait for GPS to initialize home location
    dronekit_utils.wait_for_home_location(vehicle)

    # Record GPS and realsense data
    record_data(vehicle, rs)


if __name__ == "__main__":
    main()
