import sys
sys.path.append("/usr/local/lib")
import os
# Import the libraries
import pyrealsense2 as rs
import numpy as np
import transformations as tf
import math as m
import time
import argparse
from marker_detection.camera import Realsense
from utils import file_utils

DATA_DIR = "/home/vadl/catkin_ws/src/vadl-2020/slam_evaluation/data"
#RS_FILE_BASE = "rs_pose"
#ACCEL_FILE_BASE = "rs_accel"
RS_POSE_FILE = DATA_DIR + "/" + file_utils.create_file_name_date() + ".txt"
#RS_ACCEL_FILE = file_utils.create_file_name_chronological(DATA_DIR, ACCEL_FILE_BASE, "txt")
# pose files to save to
rs_pose_file = file_utils.open_file(RS_POSE_FILE)
#rs_accel_file = file_utils.open_file(RS_ACCEL_FILE)

#######################################
# Parameters
#######################################

# 0 is sideways 45 degrees
# 1 is forward facing
# 2 is downward facing
# 3 is sideways downward, usb towards the front
REALSENSE_ORIENTATION = 3

# In NED frame, offset from the IMU or the center of gravity to the camera's origin point
body_offset_enabled = 0
body_offset_x = 0.05  # In meters (m), so 0.05 = 5cm
body_offset_y = 0  # In meters (m)
body_offset_z = 0  # In meters (m)

# Global scale factor, position x y z will be scaled up/down by this factor
scale_factor = 1.0

# Enable using yaw from compass to align north (zero degree is facing north)
compass_enabled = 0

vehicle = None
pipe = None

# pose data confidence: 0x0 - Failed / 0x1 - Low / 0x2 - Medium / 0x3 - High
pose_data_confidence_level = ('Failed', 'Low', 'Medium', 'High')

#######################################
# Parsing user' inputs
#######################################

parser = argparse.ArgumentParser(description='Reboots vehicle')
parser.add_argument('--debug_enable', type=int,
                    help="Enable debug messages on terminal")

args = parser.parse_args()
debug_enable = args.debug_enable

''' Realsense to NED pose transform, upside down and rotated 45 degrees
0: NED Origin
1: RS Origin
2: NED Frame
3: RS Frame

H0_2: NED Frame rel. to NED origin (pose sent to Pixhawk)
H0_1: RS Origin rel. to NED Origin
H1_3: RS Frame rel. to RS Origin (pose received from Realsense)
H3_2: NED Frame rel. to RS Frame

H0_2 = H0_1.dot(H1_3).dot(H3_2)

H3_2 = H3_A.dot(HA_B).dot(HB_2)
Broken up into a 180 deg. flip about z (A), 45 deg rotation about x (B) and 
left hand to right hand coord. system change.
'''
if REALSENSE_ORIENTATION == 0:

    # Upside down, rotated 45 degrees, sideways mount
    H0_1 = np.array([[1, 0, 0, 0],
                     [0, 0, 1, 0],
                     [0, -1, 0, 0],
                     [0, 0, 0, 1]])

    HA_3 = np.array([[-1, 0, 0, 0],
                     [0, -1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])
    H3_A = tf.inverse_matrix(HA_3)

    HA_B = np.array([[1, 0, 0, 0],
                     [0, np.cos(40*np.pi / 180), -np.sin(40*np.pi / 180), 0],
                     [0, np.sin(40*np.pi / 180), np.cos(40*np.pi / 180), 0],
                     [0, 0, 0, 1]])

    HB_2 = np.array([[1, 0, 0, 0],
                     [0, 0, -1, 0],
                     [0, 1, 0, 0],
                     [0, 0, 0, 1]])

    H3_2 = H3_A.dot(HA_B).dot(HB_2)
elif REALSENSE_ORIENTATION == 1:

    # Forward, USB port to the right
    H0_1 = np.array([[0, 0, -1, 0],
                     [1, 0, 0, 0],
                     [0, -1, 0, 0],
                     [0, 0, 0, 1]])
    H3_2 = np.linalg.inv(H0_1)
elif REALSENSE_ORIENTATION == 2:

    # Downfacing, USB port to the right
    H0_1 = np.array([[0, 0, 1, 0],
                     [-1, 0, 0, 0],
                     [0, -1, 0, 0],
                     [0, 0, 0, 1]])

    H3_2 = np.array([[0, -1, 0, 0],
                     [-1, 0, 0, 0],
                     [0, 0, -1, 0],
                     [0, 0, 0, 1]])
else:

    # Sideways, downward, USB portforward
    H0_1 = np.array([[1, 0, 0, 0],
                     [0, 0, 1, 0],
                     [0, -1, 0, 0],
                     [0, 0, 0, 1]])
    H3_2 = np.array([[1, 0, 0, 0],
                     [0, -1, 0, 0],
                     [0, 0, -1, 0],
                     [0, 0, 0, 1]])


def rs_to_body(H1_3):
    return (H0_1.dot(H1_3)).dot(H3_2)

if not debug_enable:
    debug_enable = 0
else:
    debug_enable = 1
    np.set_printoptions(precision=4, suppress=True)  # Format output on terminal
    print("INFO: Debug messages enabled.")

#######################################
# Main code starts here
#######################################

print("INFO: Connecting to Realsense camera.")
rs = Realsense()
print("INFO: Realsense connected.")
print("Logging data...")
start_time = time.time()
try:
    while True:

        data = rs.read()
        if data:
            # Store the timestamp for MAVLink messages
            current_time = int(round(time.time() * 1000000))

            # In transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!
            H1_3 = tf.quaternion_matrix(
                [data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z])
            H1_3[0][3] = data.translation.x * scale_factor
            H1_3[1][3] = data.translation.y * scale_factor
            H1_3[2][3] = data.translation.z * scale_factor

            # Transform to aeronautic coordinates (body AND reference frame!)
            H0_2 = rs_to_body(H1_3)

            # Take offsets from body's center of gravity (or IMU) to camera's origin into account
            if body_offset_enabled == 1:
                H_body_camera = tf.euler_matrix(0, 0, 0, 'sxyz')
                H_body_camera[0][3] = body_offset_x
                H_body_camera[1][3] = body_offset_y
                H_body_camera[2][3] = body_offset_z
                H_camera_body = np.linalg.inv(H_body_camera)
                H0_2 = H_body_camera.dot(H0_2.dot(H_camera_body))

            # Record time
            timestamp = time.time() - start_time

            # Convert to NED & Euler for data logging
            pose_ned = np.array(tf.translation_from_matrix(H0_2))

            # Write pose and accelerations to file
            rs_pose_file.write(str(timestamp) + " " +
                               str(pose_ned[0]) + " " +
                               str(pose_ned[1]) + " " +
                               str(pose_ned[2]) + " " +
                               str(data.rotation.w) + " " +
                               str(data.rotation.x) + " " +
                               str(data.rotation.y) + " " +
                               str(data.rotation.z) + "\n")
            # rs_accel_file.write(str(timestamp) + " " +
            #                     str(data.acceleration.x) + " " +
            #                     str(data.acceleration.y) + " " +
            #                     str(data.acceleration.z) + "\n")

            # Show debug messages here
            if debug_enable == 1:
                os.system('clear')  # This helps in displaying the messages to be more readable
                print("DEBUG: Raw RPY[deg]: {}".format(
                    np.array(tf.euler_from_matrix(H1_3, 'sxyz')) * 180 / m.pi))
                print("DEBUG: NED RPY[deg]: {}".format(
                    np.array(tf.euler_from_matrix(H0_2, 'sxyz')) * 180 / m.pi))
                print("DEBUG: Raw pos xyz : {}".format(
                    np.array([data.translation.x, data.translation.y, data.translation.z])))
                print("DEBUG: NED pos xyz : {}".format(np.array(tf.translation_from_matrix(H0_2))))

except KeyboardInterrupt:
    print("INFO: KeyboardInterrupt has been caught. Cleaning up...")

finally:
    pipe.stop()
    vehicle.close()
    print("INFO: Realsense pipeline and vehicle object closed.")
    sys.exit()
