#!/usr/bin/env python3

#####################################################
##          librealsense T265 to MAVLink           ##
#####################################################
# This script assumes pyrealsense2.[].so file is found under the same directory as this script
# Install required packages: 
#   pip install pyrealsense2
#   pip install transformations
#   pip3 install dronekit
#   pip3 install apscheduler

 # Set the path for IDLE
import sys
sys.path.append("/usr/local/lib/")

# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"

# Import the libraries
import pyrealsense2 as rs
import numpy as np
import transformations as tf
import math as m
import time
import argparse
import threading
from apscheduler.schedulers.background import BackgroundScheduler

from dronekit import connect, VehicleMode
from pymavlink import mavutil
from utils import dronekit_utils
from marker_detection.camera import Realsense

#######################################
# Parameters
#######################################

# Default configurations for connection to the FCU
vision_msg_hz_default = 30.0
confidence_msg_hz_default = 1

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
home_alt = 0 #TODO: Set to 0? 

vehicle = None
pipe = None

# pose data confidence: 0x0 - Failed / 0x1 - Low / 0x2 - Medium / 0x3 - High 
pose_data_confidence_level = ('Failed', 'Low', 'Medium', 'High')

#######################################
# Parsing user' inputs
#######################################

parser = argparse.ArgumentParser(description='Reboots vehicle')
parser.add_argument('--vision_msg_hz', type=float,
                    help="Update frequency for VISION_POSITION_ESTIMATE message. If not specified, a default value will be used.")
parser.add_argument('--confidence_msg_hz', type=float,
                    help="Update frequency for confidence level. If not specified, a default value will be used.")
parser.add_argument('--scale_calib_enable', type=bool,
                    help="Scale calibration. Only run while NOT in flight")
parser.add_argument('--debug_enable',type=int,
                    help="Enable debug messages on terminal")

args = parser.parse_args()

vision_msg_hz = args.vision_msg_hz
confidence_msg_hz = args.confidence_msg_hz
scale_calib_enable = args.scale_calib_enable
debug_enable = args.debug_enable

if not vision_msg_hz:
    vision_msg_hz = vision_msg_hz_default
    print("INFO: Using default vision_msg_hz", vision_msg_hz)
else:
    print("INFO: Using vision_msg_hz", vision_msg_hz)
if not confidence_msg_hz:
    confidence_msg_hz = confidence_msg_hz_default
    print("INFO: Using default confidence_msg_hz", confidence_msg_hz)
else:
    print("INFO: Using confidence_msg_hz", confidence_msg_hz)
if body_offset_enabled == 1:
    print("INFO: Using camera position offset: Enabled, x y z is", body_offset_x, body_offset_y, body_offset_z)
else:
    print("INFO: Using camera position offset: Disabled")
if compass_enabled == 1:
    print("INFO: Using compass: Enabled. Heading will be aligned to north.")
else:
    print("INFO: Using compass: Disabled")
if scale_calib_enable:
    print("\nINFO: SCALE CALIBRATION PROCESS. DO NOT RUN DURING FLIGHT.\nINFO: TYPE IN NEW SCALE IN FLOATING POINT FORMAT\n")
else:
    if scale_factor == 1.0:
        print("INFO: Using default scale factor", scale_factor)
    else:
        print("INFO: Using scale factor", scale_factor)
if not debug_enable:
    debug_enable = 0
else:
    debug_enable = 1
    np.set_printoptions(precision=4, suppress=True) # Format output on terminal
    print("INFO: Debug messages enabled.")

''' Realsense to NED pose transform
0: NED Origin
1: RS Origin
2: NED Frame
3: RS Frame

H2_0: NED Frame rel. to NED origin (pose sent to Pixhawk)
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


#######################################
# Functions
#######################################

# TODO: Make a MavlinkBackgroundScheduler class to abstract Mavlink message sending
# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
def send_vision_position_message():
    global current_time, H0_2

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
    global data, current_confidence
    if data is not None:
        # Show confidence level on terminal
        print("INFO: Tracking confidence: ", pose_data_confidence_level[data.tracker_confidence])

        # Send MAVLink message to show confidence level numerically
        msg = vehicle.message_factory.vision_position_delta_encode(
            0,	            #us	Timestamp (UNIX time or time since system boot)
            0,	            #Time since last reported camera frame
            [0, 0, 0],      #angle_delta
            [0, 0, 0],      #position_delta
            float(data.tracker_confidence * 100 / 3)          
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()

        # If confidence level changes, send MAVLink message to show confidence level textually and phonetically
        if current_confidence is None or current_confidence != data.tracker_confidence:
            current_confidence = data.tracker_confidence
            confidence_status_string = 'Tracking confidence: ' + pose_data_confidence_level[data.tracker_confidence]
            status_msg = vehicle.message_factory.statustext_encode(
                3,	            #severity, defined here: https://mavlink.io/en/messages/common.html#MAV_SEVERITY, 3 will let the message be displayed on Mission Planner HUD
                confidence_status_string.encode()	  #text	char[50]       
            )
            vehicle.send_mavlink(status_msg)
            vehicle.flush()

# Listen to messages that indicate EKF is ready to set home, then set EKF home automatically.
def statustext_callback(self, attr_name, value):
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
      
# Code to TRICK THE DUMBASS ARDUCOPTER DEV CODE

def trick_compass(old_vehicle):
    if old_vehicle.parameters['MAG_ENABLE'] == 0 or old_vehicle.parameters['COMPASS_USE'] == 0:
        # Enable the compass and the EKF's use of it for heading
        old_vehicle.parameters['COMPASS_USE'] = 1
        old_vehicle.parameters['MAG_ENABLE'] = 1
        print("Enabling MAG_ENABLE and COMPASS_USE")

        # Reboot and reconnect to the vehicle
        vehicle = dronekit_utils.reboot_and_connect(old_vehicle, args)

    # Arm and disarm
    dronekit_utils.arm_no_failsafe(vehicle)
    while not vehicle.armed:
        print("Waiting for arming")
        time.sleep(1)
    dronekit_utils.disarm(vehicle)

    # Disable compass parameters; TODO: no reboot required?
    vehicle.parameters['MAG_ENABLE'] = 0
    vehicle.parameters['COMPASS_USE'] = 0
    print("MAG_ENABLE and COMPASS_USE set to 0")

    return vehicle

#######################################
# Main code starts here
#######################################

print("INFO: Connecting to Realsense camera.")
rs = Realsense()
print("INFO: Realsense connected.")

print("INFO: Connecting to vehicle.")
vehicle = dronekit_utils.connect_vehicle()
print("INFO: Vehicle connected.")

# print("INFO: Tricking compass.")
#vehicle = trick_compass(vehicle)
# print("INFO: Compass tricked.")

# Listen to the mavlink messages that will be used as trigger to set EKF home automatically
vehicle.add_message_listener('STATUSTEXT', statustext_callback)

if compass_enabled == 1:
    # Listen to the attitude data in aeronautical frame
    vehicle.add_message_listener('ATTITUDE', att_msg_callback)

data = None
current_confidence = None
H_aeroRef_aeroBody = None
heading_north_yaw = None

# Send MAVlink messages in the background
sched = BackgroundScheduler()
sched.add_job(send_vision_position_message, 'interval', seconds = 1/vision_msg_hz)
sched.add_job(send_confidence_level_dummy_message, 'interval', seconds = 1/confidence_msg_hz)

# For scale calibration, we will use a thread to monitor user input
if scale_calib_enable == True:
    scale_update_thread = threading.Thread(target=scale_update)
    scale_update_thread.daemon = True
    scale_update_thread.start()

sched.start()

if compass_enabled == 1:
    # Wait a short while for yaw to be correctly initiated
    time.sleep(1)

print("INFO: Sending VISION_POSITION_ESTIMATE messages to FCU.")
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

            #cmds = vehicle.commands
            #cmds.download()
            #cmds.wait_ready()

            print("Altitude (g): %s" % vehicle.location.global_frame.alt)
            print("Altitude (gr): %s" % vehicle.location.global_relative_frame.alt)
            print("Altitude (l): %s" % vehicle.location.local_frame.down)
            print("Home Location: %s" % vehicle.home_location)

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
    rs.stop()
    vehicle.close()
    print("INFO: Realsense pipeline and vehicle object closed.")
    sys.exit()
