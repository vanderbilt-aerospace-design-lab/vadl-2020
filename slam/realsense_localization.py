### --------------------------------------------------------------------------------------------- ###
# This script receives position estimates from the Intel Realsense T265 sensor 
# and sends them to the Pixhawk. The Pixhawk can then use these pose estimates to 
# localize and achieve extremely stable flight. 

# Pose estimates are sent to the Pixhawk using the VISION_POSITION_ESTIMATE MAVLink message.
# Messages are sent at 30 Hz so to not flood the Pixhawk with messages.  

# The Pixhawk must be configured for SLAM flight. Refer to the README in this section
# for instructions.

# This script has been modified based on https://github.com/thien94/vision_to_mavros/blob/master/scripts/t265_to_mavlink.py

# These tutorials are fantastic for getting the Pixhawk and Realsense set up:
# Getting Started: https://discuss.ardupilot.org/t/integration-of-ardupilot-and-vio-tracking-camera-part-1-getting-started-with-the-intel-realsense-t265-on-rasberry-pi-3b/43162
# Realsense -> Pixhawk communication: https://discuss.ardupilot.org/t/integration-of-ardupilot-and-vio-tracking-camera-part-4-non-ros-bridge-to-mavlink-in-python/44001
# ROS version: https://discuss.ardupilot.org/t/integration-of-ardupilot-and-vio-tracking-camera-part-2-complete-installation-and-indoor-non-gps-flights/43405

# The other parts of those tutorials are also helpful for debugging problems and additional info.

### --------------------------------------------------------------------------------------------- ###


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

# Custom libraries
from utils import dronekit_utils, file_utils
from marker_detection.camera import Realsense

DATA_DIR = "slam/data" # Save pose and vibration data
RS_FILE_BASE = "rs_pose" # Base pose file name
ACCEL_FILE_BASE = "rs_accel" # Base vibration data file name

# Create pose and vibration files
RS_POSE_FILE = file_utils.create_file_name_chronological(DATA_DIR, RS_FILE_BASE, "txt")
RS_ACCEL_FILE = file_utils.create_file_name_chronological(DATA_DIR, ACCEL_FILE_BASE, "txt")
rs_pose_file = file_utils.open_file(RS_POSE_FILE)
rs_accel_file = file_utils.open_file(RS_ACCEL_FILE)

#######################################
# Parameters
#######################################

## Realsense Orientation ##
# This parameter controls the pose transform used to convert the pose estimate
# in the Realsense reference frame to the UAV body frame. It depends on how the sensor
# is mounted on the UAV. We have found the best results with forward facing and downward facing, 
# and tracking failure in large-scale environments at 45 degrees.
# 0: Sideways (rel. to UAV front) 45 degrees, USB towards the front
# 1 is forward facing (cameras perpendicular to ground), USB towards the right
# 2 is downward facing (cameras facing the ground), USB towards the right
# 3 is sideways downward, USB towards the front
REALSENSE_ORIENTATION = 3

# Default configurations for connection to the Pixhawk FCU
vision_msg_hz = 30.0
confidence_msg_hz = 1 

# In NED frame, offset from the IMU or the center of gravity to the Realsense origin point
body_offset_enabled = 0
body_offset_x = 0.05    # In meters (m), so 0.05 = 5cm
body_offset_y = 0       # In meters (m)
body_offset_z = 0       # In meters (m)

# Global scale factor, position x y z will be scaled up/down by this factor
# NOTE: This has not been tested by VADL, but will probably be required for 
# long-distance flight in outdoor environments. There is a lot of pose drift over time,
# so it would be a good idea to empirically determine the drift over different distances traveled
# and create a variable scale factor.
scale_factor = 1.0

# Enable using yaw from compass to align north (zero degree is facing north)
# This is not necessary, but is helpful if you would like to visualize the UAV's trajectory
# on a Ground Control Station like Mission Planner.
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

vehicle = None # Pixhawk object
H0_2 = None # Store the pose sent to the Pixhawk
current_time = None
data = None # Data packet received from the Realsense
current_confidence = None # How confident the Realsense is in its pose estimate

## Realsense to NED pose transformations ##
# 0: NED Origin
# 1: Realsense (RS) Origin
# 2: NED Frame
# 3: RS Frame
#
# H0_2: NED Frame rel. to NED origin (this is the final pose sent to the Pixhawk)
# H0_1: RS Origin rel. to NED Origin
# H1_3: RS Frame rel. to RS Origin (pose received from Realsense)
# H3_2: NED Frame rel. to RS Frame
#
# H0_2 = H0_1.dot(H1_3).dot(H3_2)
#
# With SLAM flight enabled and GPS disabled, NED and body frame are equivalent, since the
# UAV has no idea of true north. NED origin is defined based on the starting orientation of the UAV.
if REALSENSE_ORIENTATION == 0:

    # Upside down, rotated 40 degrees, sideways mount
    # H3_2 = H3_A.dot(HA_B).dot(HB_2)
    # This breaks down into a 180 deg. flip about z (A), 45 deg rotation about x (B) and 
    # left hand to right hand coord. system change.

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

    # Forward-facing, USB port to the right
    H0_1 = np.array([[0, 0, -1, 0],
                     [1, 0, 0, 0],
                     [0, -1, 0, 0],
                     [0, 0, 0, 1]])

    H3_2 = np.linalg.inv(H0_1)

elif REALSENSE_ORIENTATION == 2:

    # Downward-facing, USB port to the left
    H0_1 = np.array([[0, 0, 1, 0],
                     [-1, 0, 0, 0],
                     [0, -1, 0, 0],
                     [0, 0, 0, 1]])

    H3_2 = np.array([[0, -1, 0, 0],
                     [-1, 0, 0, 0],
                     [0, 0, -1, 0],
                     [0, 0, 0, 1]])
else:

    # Downward-facing, USB port forward
    H0_1 = np.array([[1, 0, 0, 0],
                     [0, 0, 1, 0],
                     [0, -1, 0, 0],
                     [0, 0, 0, 1]])
    H3_2 = np.array([[1, 0, 0, 0],
                     [0, -1, 0, 0],
                     [0, 0, -1, 0],
                     [0, 0, 0, 1]])


# Transform the pose received from the Realsense into the NED reference frame.
# 
# H1_3: Pose received from the Realsense
# Returns: Pose in NED frame rel. to NED origin
def rs_to_body(H1_3):
    return (H0_1.dot(H1_3)).dot(H3_2)


#######################################
# Functions
#######################################

# When SLAM flight is enabled on the Pixhawk, it is expecting to receive 
# VISION_POSITION_ESTIMATE messages. All you need to do is send these messages
# and the UAV will perform stable hover.
#
# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
def send_vision_position_message():

    global vehicle

    if H0_2 is not None:

        # Convert rotation matrix to Euler angles
        rpy_rad = np.array( tf.euler_from_matrix(H0_2, 'sxyz'))

        # Create vision pose message
        msg = vehicle.message_factory.vision_position_estimate_encode(
            current_time,       # us Timestamp (UNIX time or time since system boot)
            H0_2[0][3],	        # NED X position
            H0_2[1][3],         # NED Y position
            H0_2[2][3],	        # NED Z position
            rpy_rad[0],	        # Roll angle
            rpy_rad[1],	        # Pitch angle
            rpy_rad[2]	        # Yaw angle
        )

        # Send the message
        vehicle.send_mavlink(msg)
        vehicle.flush()

# For a lack of a dedicated message, we pack the confidence level into a message that will not be used, so we can view it on GCS
# Confidence level value: 0 - 3, remapped to 0 - 100: 0% - Failed / 33.3% - Low / 66.6% - Medium / 100% - High
# This is not necessary and can be commented out.
def send_confidence_level_dummy_message():

    global current_confidence, vehicle

    if data is not None:

        # Print confidence level to terminal
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
# This is a way to dynamically change the scale factor during flight
# NOTE: This has not been tested by VADL and may be dangerous to do during flight.
# Proceed with caution.
def scale_update():

    global scale_factor

    while True:
        scale_factor = float(input("INFO: Type in new scale as float number\n"))
        print("INFO: New scale is ", scale_factor)


# Receive pose estimates from the Realsense and send them to the Pixhawk.
def localize(rs, sched=None):

    # Global variables are usually not good style, but are required here (for now)
    global vehicle, H0_2, data, current_time

    # Listen to the mavlink messages that will be used as trigger to set the EKF home automatically
    # The Pixhawk will not allow arming if the EKF and Home origins have not been set. Once data is
    # being received from the Realsense, this callback will set the origins.
    # NOTE: There has been some trouble with this; sometimes it fails to set the origin. If this happens,
    # reboot the Pixhawk until the issue resolves itself (or find a better solution!)
    vehicle.add_message_listener('STATUSTEXT', statustext_callback)

    if compass_enabled == 1:

        # Listen to the attitude data in aeronautical frame
        vehicle.add_message_listener('ATTITUDE', att_msg_callback)

    # Send MAVlink messages in the background
    if sched is None:
        sched = BackgroundScheduler()

    # Add message senders for vision pose message and confidence message
    sched.add_job(send_vision_position_message, 'interval', seconds=1 / vision_msg_hz, max_instances=5)
    sched.add_job(send_confidence_level_dummy_message, 'interval', seconds=1 / confidence_msg_hz)

    # For scale calibration, we will use a thread to monitor user input
    # NOTE: not tested by VADL
    if scale_calib_enable:
        scale_update_thread = threading.Thread(target=scale_update)
        scale_update_thread.daemon = True
        scale_update_thread.start()

    # Start sending background messages to Mavlink
    sched.start()

    if compass_enabled == 1:
        # Wait a short while for yaw to be correctly initiated
        time.sleep(1)

    # This function is contained in its own thread and runs indefinitely.
    start_time = time.time()
    print("Sending VISION_POSITION_ESTIMATE messages to FCU.")
    try:
        while True:

            # Obtain pose from the Realsense
            data = rs.read()

            if data:

                # Store the timestamp for MAVLink messages
                current_time = int(round(time.time() * 1000000))

                # Extract pose info and format as a homogeneous matrix.
                # In transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]
                H1_3 = tf.quaternion_matrix(
                    [data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z])
                H1_3[0][3] = data.translation.x * scale_factor
                H1_3[1][3] = data.translation.y * scale_factor
                H1_3[2][3] = data.translation.z * scale_factor

                # Transform to NED coordinates (body AND reference frame!)
                H0_2 = rs_to_body(H1_3)

                # Account for offsets from body's center of gravity (or IMU) to camera's origin
                if body_offset_enabled == 1:
                    H_body_camera = tf.euler_matrix(0, 0, 0, 'sxyz')
                    H_body_camera[0][3] = body_offset_x
                    H_body_camera[1][3] = body_offset_y
                    H_body_camera[2][3] = body_offset_z
                    H_camera_body = np.linalg.inv(H_body_camera)

                    # Shift the NED pose by an offset
                    H0_2 = H_body_camera.dot(H0_2.dot(H_camera_body))

                # Record time
                timestamp = time.time() - start_time

                # Convert to NED & quaternion for data logging
                pose_ned = np.array(tf.translation_from_matrix(H0_2))

                # Write pose and accelerations to file
                # These logs follow the .tum format so they can be processed by
                # the EVO SLAM comparisons package:
                #
                # https://github.com/MichaelGrupp/evo
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

        # Clean up Realsense and Vehicle objects
        rs.stop()
        vehicle.close()

        print("Realsense pipeline and vehicle object closed.")
        sys.exit()


# Start a thread that sends Realsense pose estimates to the Pixhawk in the background.
def start(vehicle_object, rs=None, scheduler=None):

    global vehicle
    vehicle = vehicle_object

    # Create realsense object
    if rs is None:
        rs = Realsense()

    # Spawn a thread to transform realsense pose to UAV pose and send vision_position_estimate messages to
    # Mavlink in the background
    # Make it daemon so it ends when the program terminates
    localize_thread = Thread(target=localize, args=(rs, scheduler))
    localize_thread.daemon = True
    localize_thread.start() # Start the thread
