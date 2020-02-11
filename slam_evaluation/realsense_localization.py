 # Set the path for IDLE
import sys
sys.path.append("/usr/local/lib/")

# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"

# Import the libraries
import numpy as np
import transformations as tf
import math as m
import time
import threading
from threading import Thread
from apscheduler.schedulers.background import BackgroundScheduler

from utils import dronekit_utils, file_utils
from marker_detection.camera import Realsense

DATA_DIR = "./slam_evaluation/data"
RS_FILE_BASE = "rs_pose"
ACCEL_FILE_BASE = "rs_accel"
RS_POSE_FILE = file_utils.create_file_name_chronological(DATA_DIR, RS_FILE_BASE, "txt")
RS_ACCEL_FILE = file_utils.create_file_name_chronological(DATA_DIR, ACCEL_FILE_BASE, "txt")
# pose files to save to
rs_pose_file = file_utils.open_file(RS_POSE_FILE)
rs_accel_file = file_utils.open_file(RS_ACCEL_FILE)

#######################################
# Parameters
#######################################

# Default configurations for connection to the FCU
vision_msg_hz = 30.0
confidence_msg_hz = 1

# In NED frame, offset from the IMU or the center of gravity to the camera's origin point
body_offset_enabled = 0
body_offset_x = 0.05    # In meters (m), so 0.05 = 5cm
body_offset_y = 0       # In meters (m)
body_offset_z = 0       # In meters (m)

# Global scale factor, position x y z will be scaled up/down by this factor
scale_factor = 1.0

# Enable using yaw from compass to align north (zero degree is facing north)
compass_enabled = 0

# Default global position of home/ origin
#home_lat = 341613615
home_lat = 0
home_lon = 0
#home_lon = -1185031700
home_alt = 0

scale_calib_enable = 0
debug_enable = 0

# pose data confidence: 0x0 - Failed / 0x1 - Low / 0x2 - Medium / 0x3 - High
pose_data_confidence_level = ('Failed', 'Low', 'Medium', 'High')

''' Realsense to NED pose transform
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

vehicle = None
H0_2 = None
current_time = None
data = None
current_confidence = None

#######################################
# Functions
#######################################

# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
def send_vision_position_message():
    global vehicle
    if H0_2 is not None:
        rpy_rad = np.array( tf.euler_from_matrix(H0_2, 'sxyz'))

        msg = vehicle.message_factory.vision_position_estimate_encode(
            current_time,                       # us Timestamp (UNIX time or time since system boot)
            H0_2[0][3],	        # Global X position
            H0_2[1][3],           # Global Y position
            H0_2[2][3],	        # Global Z position
            rpy_rad[0],	                        # Roll angle
            rpy_rad[1],	                        # Pitch angle
            rpy_rad[2]	                        # Yaw angle
        )

        vehicle.send_mavlink(msg)
        vehicle.flush()

# For a lack of a dedicated message, we pack the confidence level into a message that will not be used, so we can view it on GCS
# Confidence level value: 0 - 3, remapped to 0 - 100: 0% - Failed / 33.3% - Low / 66.6% - Medium / 100% - High
def send_confidence_level_dummy_message():
    global current_confidence, vehicle
    if data is not None:
        # Show confidence level on terminal
        print("INFO: Tracking confidence: ", pose_data_confidence_level[data.tracker_confidence])

        # Send MAVLink message to show confidence level numerically
        # msg = vehicle.message_factory.vision_position_delta_encode(
        #     0,	            #us	Timestamp (UNIX time or time since system boot)
        #     0,	            #Time since last reported camera frame
        #     [0, 0, 0],      #angle_delta
        #     [0, 0, 0],      #position_delta
        #     float(data.tracker_confidence * 100 / 3)
        # )
        # vehicle.send_mavlink(msg)
        # vehicle.flush()

        # If confidence level changes, send MAVLink message to show confidence level textually and phonetically
        # if current_confidence is None or current_confidence != data.tracker_confidence:
        #     current_confidence = data.tracker_confidence
            # confidence_status_string = 'Tracking confidence: ' + pose_data_confidence_level[data.tracker_confidence]
            # status_msg = vehicle.message_factory.statustext_encode(
            #     3,	            #severity, defined here: https://mavlink.io/en/messages/common.html#MAV_SEVERITY, 3 will let the message be displayed on Mission Planner HUD
            #     confidence_status_string.encode()	  #text	char[50]
            # )
            # vehicle.send_mavlink(status_msg)
            # vehicle.flush()

# Listen to messages that indicate EKF is ready to set home, then set EKF home automatically.
def statustext_callback(self, attr_name, value):
    global vehicle
    # These are the status texts that indicates EKF is ready to receive home position
    if value.text == "GPS Glitch" or value.text == "GPS Glitch cleared" or value.text == "EKF2 IMU1 ext nav yaw alignment complete":
        time.sleep(0.1)
        print("INFO: Set EKF home with default GPS location")
        dronekit_utils.set_default_global_origin(vehicle, home_lat, home_lon, home_alt)
        dronekit_utils.set_default_home_position(vehicle, home_lat, home_lon, home_alt)

# Listen to attitude data to acquire heading when compass data is enabled
def att_msg_callback(self, attr_name, value):
    global heading_north_yaw
    if heading_north_yaw is None:
        heading_north_yaw = value.yaw
        print("INFO: Received first ATTITUDE message with heading yaw", heading_north_yaw * 180 / m.pi, "degrees")
    else:
        heading_north_yaw = value.yaw
        print("INFO: Received ATTITUDE message with heading yaw", heading_north_yaw * 180 / m.pi, "degrees")

# Monitor user input from the terminal and update scale factor accordingly
def scale_update():
    global scale_factor
    while True:
        scale_factor = float(input("INFO: Type in new scale as float number\n"))
        print("INFO: New scale is ", scale_factor)

def localize(rs, sched=None):
    global vehicle, H0_2, data, current_time

    # Listen to the mavlink messages that will be used as trigger to set EKF home automatically
    vehicle.add_message_listener('STATUSTEXT', statustext_callback)

    if compass_enabled == 1:
        # Listen to the attitude data in aeronautical frame
        vehicle.add_message_listener('ATTITUDE', att_msg_callback)

    # Send MAVlink messages in the background
    if sched is None:
        sched = BackgroundScheduler()

    sched.add_job(send_vision_position_message, 'interval', seconds=1 / vision_msg_hz, max_instances=5)
    sched.add_job(send_confidence_level_dummy_message, 'interval', seconds=1 / confidence_msg_hz)

    # For scale calibration, we will use a thread to monitor user input
    if scale_calib_enable:
        scale_update_thread = threading.Thread(target=scale_update)
        scale_update_thread.daemon = True
        scale_update_thread.start()

    # Start sending background messages to Mavlink
    sched.start()

    if compass_enabled == 1:
        # Wait a short while for yaw to be correctly initiated
        time.sleep(1)

    start_time = time.time()
    print("Sending VISION_POSITION_ESTIMATE messages to FCU.")
    try:
        while True:

            # Obtain pose from the Realsense
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
                H0_2 = (H0_1.dot(H1_3)).dot(H3_2)

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
                rs_accel_file.write(str(timestamp) + " " +
                                    str(data.acceleration.x) + " " +
                                    str(data.acceleration.y) + " " +
                                    str(data.acceleration.z) + "\n")

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
        print("EXCEPTION: KeyboardInterrupt has been caught. Cleaning up...")

    finally:
        rs.stop()
        vehicle.close()
        print("Realsense pipeline and vehicle object closed.")
        sys.exit()

def start(vehicle_object, scheduler=None):
    global vehicle
    vehicle = vehicle_object
    rs = Realsense()

    # Spawn a thread to transform realsense pose to UAV pose and send vision_position_estimate messages to
    # Mavlink in the background
    # Make it daemon so it ends with the program ending
    localize_thread = Thread(target=localize, args=(rs, scheduler))
    localize_thread.daemon = True
    localize_thread.start()
