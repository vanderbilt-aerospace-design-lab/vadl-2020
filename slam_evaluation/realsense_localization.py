# Set the path for IDLE
import sys
sys.path.append("/usr/local/lib/")

# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"

# Libraries
import numpy as np
import transformations as tf
import math as m
import time
import threading
from apscheduler.schedulers.background import BackgroundScheduler

# Personal files
from marker_detection.camera import Realsense
from utils import dronekit_utils, file_utils
from threading import Thread

# Create file names for debugging
DATA_DIR = "./slam_evaluation/self.data"
RS_FILE_BASE = "rs_pose"
ACCEL_FILE_BASE = "rs_accel"
RS_POSE_FILE = file_utils.create_file_name_chronological(DATA_DIR, RS_FILE_BASE, "txt")
RS_ACCEL_FILE = file_utils.create_file_name_chronological(DATA_DIR, ACCEL_FILE_BASE, "txt")

# open files
rs_pose_file = file_utils.open_file(RS_POSE_FILE)
rs_accel_file = file_utils.open_file(RS_ACCEL_FILE)

''' Realsense to NED pose transform
    0: NED Origin
    1: RS Origin
    2: NED Frame
    3: RS Frame

    self.H0_2: NED Frame rel. to NED origin (pose sent to Pixhawk)
    H0_1: RS Origin rel. to NED Origin
    H1_3: RS Frame rel. to RS Origin (pose received from Realsense)
    H3_2: NED Frame rel. to RS Frame

    self.H0_2 = H0_1.dot(H1_3).dot(H3_2)

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
                 [0, np.cos(40 * np.pi / 180), -np.sin(40 * np.pi / 180), 0],
                 [0, np.sin(40 * np.pi / 180), np.cos(40 * np.pi / 180), 0],
                 [0, 0, 0, 1]])

HB_2 = np.array([[1, 0, 0, 0],
                 [0, 0, -1, 0],
                 [0, 1, 0, 0],
                 [0, 0, 0, 1]])

H3_2 = H3_A.dot(HA_B).dot(HB_2)

data_global = None
vehicle_global = None
heading_north_yaw = None
H0_2_global = None
home_lat = 0
home_lon = 0
home_alt = 0
# pose self.data confidence: 0x0 - Failed / 0x1 - Low / 0x2 - Medium / 0x3 - High
pose_data_confidence_level = ('Failed', 'Low', 'Medium', 'High')
current_confidence = None

# Listen to messages that indicate EKF is ready to set home, then set EKF home automatically.
def statustext_callback(self, attr_name, value):
    # These are the status texts that indicates EKF is ready to receive home position
    if value.text == "GPS Glitch" or value.text == "GPS Glitch cleared" or value.text == "EKF2 IMU1 ext nav yaw alignment complete":
        time.sleep(0.1)
        print("INFO: Set EKF home with default GPS location")
        dronekit_utils.set_default_global_origin(vehicle_global, home_lat, home_lon, home_alt)
        dronekit_utils.set_default_home_position(vehicle_global, home_lat, home_lon, home_alt)

# Listen to attitude data to acquire heading when compass data is enabled
def att_msg_callback(self, attr_name, value):
    global heading_north_yaw
    if heading_north_yaw is None:
        heading_north_yaw = value.yaw
        print("INFO: Received first ATTITUDE message with heading yaw", heading_north_yaw * 180 / m.pi, "degrees")
    else:
        heading_north_yaw = value.yaw
        print("INFO: Received ATTITUDE message with heading yaw", heading_north_yaw * 180 / m.pi, "degrees")

# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
def send_vision_position_message():
    global H0_2_global, current_time
    if H0_2_global is not None:
        rpy_rad = np.array(tf.euler_from_matrix(H0_2_global, 'sxyz'))

        msg = vehicle_global.message_factory.vision_position_estimate_encode(
            current_time,  # us Timestamp (UNIX time or time since system boot)
            H0_2_global[0][3],  # Global X position
            H0_2_global[1][3],  # Global Y position
            H0_2_global[2][3],  # Global Z position
            rpy_rad[0],  # Roll angle
            rpy_rad[1],  # Pitch angle
            rpy_rad[2]  # Yaw angle
        )

        vehicle_global.send_mavlink(msg)
        vehicle_global.flush()

# For a lack of a dedicated message, we pack the confidence level into a message that will not be used, so we can view it on GCS
# Confidence level value: 0 - 3, remapped to 0 - 100: 0% - Failed / 33.3% - Low / 66.6% - Medium / 100% - High
def send_confidence_level_dummy_message():
    global current_confidence
    if data_global is not None:
        # Show confidence level on terminal
        print("INFO: Tracking confidence: ", pose_data_confidence_level[data_global.tracker_confidence])

        # Send MAVLink message to show confidence level numerically
        msg = vehicle_global.message_factory.vision_position_delta_encode(
            0,  # us	Timestamp (UNIX time or time since system boot)
            0,  # Time since last reported camera frame
            [0, 0, 0],  # angle_delta
            [0, 0, 0],  # position_delta
            float(data_global.tracker_confidence * 100 / 3)
        )
        vehicle_global.send_mavlink(msg)
        vehicle_global.flush()

        # If confidence level changes, send MAVLink message to show confidence level textually and phonetically
        if current_confidence is None or current_confidence != data_global.tracker_confidence:
            current_confidence = data_global.tracker_confidence
            confidence_status_string = 'Tracking confidence: ' + pose_data_confidence_level[data_global.tracker_confidence]
            status_msg = vehicle_global.message_factory.statustext_encode(
                3,
                # severity, defined here: https://mavlink.io/en/messages/common.html#MAV_SEVERITY, 3 will let the message be displayed on Mission Planner HUD
                confidence_status_string.encode()  # text	char[50]
            )
            vehicle_global.send_mavlink(status_msg)
            vehicle_global.flush()



# TODO: Add documentation once you know this works
class RealsenseLocalization:
    def __init__(self,
                 vehicle=None):

        # Default configurations for connection to the FCU
        self.vision_msg_hz = 30.0
        self.confidence_msg_hz = 1

        # In NED frame, offset from the IMU or the center of gravity to the camera's origin point
        self.body_offset_enabled = 0
        self.body_offset_x = 0.05  # In meters (m), so 0.05 = 5cm
        self.body_offset_y = 0  # In meters (m)
        self.body_offset_z = 0  # In meters (m)

        # Enable using yaw from compass to align north (zero degree is facing north)
        self.compass_enabled = 0
        self.heading_north_yaw = None

        # Global scale factor, position x y z will be scaled up/down by this factor
        self.scale_factor = 1.0
        # TODO: Figure out how to actually use this
        self.scale_calib_enable = 0

        self.debug_enable = 0

        # Realsense self.data
        self.data = None
        # Final UAV pose to send to Mavlink
        self.H0_2 = None

        # Initialize Realsense self.data collector
        self.rs = Realsense()

        # Connect to self.vehicle
        self.vehicle = vehicle
        if self.vehicle is None:
            self.vehicle = dronekit_utils.connect_vehicle()

        # Unfortunate thing we must do to use Mavlink callback functions
        global vehicle_global
        vehicle_global = self.vehicle

    def start(self):
        # Spawn a thread to transform realsense pose to UAV pose and send vision_position_estimate messages to
        # Mavlink in the background
        # Make it daemon so it ends with the program ending
        localize_thread = Thread(target=self.localize, args=())
        localize_thread.daemon = True
        localize_thread.start()


    def localize(self):

        # Listen to the mavlink messages that will be used as trigger to set EKF home automatically
        self.vehicle.add_message_listener('STATUSTEXT', statustext_callback)

        if self.compass_enabled == 1:
            # Listen to the attitude self.data in aeronautical frame
            self.vehicle.add_message_listener('ATTITUDE', att_msg_callback)

        # Send MAVlink messages in the background
        sched = BackgroundScheduler()
        sched.add_job(self.send_vision_position_message, 'interval', seconds=1 / self.vision_msg_hz)
        sched.add_job(self.send_confidence_level_dummy_message, 'interval', seconds=1 / self.confidence_msg_hz)

        # For scale calibration, we will use a thread to monitor user input
        if self.scale_calib_enable:
            scale_update_thread = threading.Thread(target=self.scale_update)
            scale_update_thread.daemon = True
            scale_update_thread.start()

        # Start sending background messages to Mavlink
        sched.start()

        if self.compass_enabled == 1:
            # Wait a short while for yaw to be correctly initiated
            time.sleep(1)

        start_time = time.time()
        print("Sending VISION_POSITION_ESTIMATE messages to FCU.")
        try:
            while True:

                # Obtain pose from the Realsense
                self.data = self.rs.read()

                global data_global
                data_global = self.data

                if self.data:
                    # Store the timestamp for MAVLink messages
                    current_time = int(round(time.time() * 1000000))

                    # In transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!
                    H1_3 = tf.quaternion_matrix(
                        [self.data.rotation.w, self.data.rotation.x, self.data.rotation.y, self.data.rotation.z])
                    H1_3[0][3] = self.data.translation.x * self.scale_factor
                    H1_3[1][3] = self.data.translation.y * self.scale_factor
                    H1_3[2][3] = self.data.translation.z * self.scale_factor

                    # Transform to aeronautic coordinates (body AND reference frame!)
                    self.H0_2 = (H0_1.dot(H1_3)).dot(H3_2)

                    # TODO: Maybe there is a better way to do this?
                    global H0_2_global
                    H0_2_global = self.H0_2

                    # Take offsets from body's center of gravity (or IMU) to camera's origin into account
                    if self.body_offset_enabled == 1:
                        H_body_camera = tf.euler_matrix(0, 0, 0, 'sxyz')
                        H_body_camera[0][3] = self.body_offset_x
                        H_body_camera[1][3] = self.body_offset_y
                        H_body_camera[2][3] = self.body_offset_z
                        H_camera_body = np.linalg.inv(H_body_camera)
                        self.H0_2 = H_body_camera.dot(self.H0_2.dot(H_camera_body))

                    # Record time
                    timestamp = time.time() - start_time

                    # Convert to NED & Euler for self.data logging
                    pose_ned = np.array(tf.translation_from_matrix(self.H0_2))

                    # Write pose and accelerations to file
                    rs_pose_file.write(str(timestamp) + " " +
                                       str(pose_ned[0]) + " " +
                                       str(pose_ned[1]) + " " +
                                       str(pose_ned[2]) + " " +
                                       str(self.data.rotation.w) + " " +
                                       str(self.data.rotation.x) + " " +
                                       str(self.data.rotation.y) + " " +
                                       str(self.data.rotation.z) + "\n")
                    rs_accel_file.write(str(timestamp) + " " +
                                        str(self.data.acceleration.x) + " " +
                                        str(self.data.acceleration.y) + " " +
                                        str(self.data.acceleration.z) + "\n")

                    # Show debug messages here
                    if self.debug_enable == 1:
                        os.system('clear')  # This helps in displaying the messages to be more readable
                        print("DEBUG: Raw RPY[deg]: {}".format(
                            np.array(tf.euler_from_matrix(H1_3, 'sxyz')) * 180 / m.pi))
                        print("DEBUG: NED RPY[deg]: {}".format(
                            np.array(tf.euler_from_matrix(self.H0_2, 'sxyz')) * 180 / m.pi))
                        print("DEBUG: Raw pos xyz : {}".format(
                            np.array([self.data.translation.x, self.data.translation.y, self.data.translation.z])))
                        print("DEBUG: NED pos xyz : {}".format(np.array(tf.translation_from_matrix(self.H0_2))))

        except KeyboardInterrupt:
            print("EXCEPTION: KeyboardInterrupt has been caught. Cleaning up...")

        finally:
            self.rs.stop()
            self.vehicle.close()
            print("Realsense pipeline and self.vehicle object closed.")
            sys.exit()

    # Monitor user input from the terminal and update scale factor accordingly
    def scale_update(self):
        while True:
            self.scale_factor = float(input("INFO: Type in new scale as float number\n"))
            print("INFO: New scale is ", self.scale_factor)
