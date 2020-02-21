# ---------------------------------------------------------------------------------- #
#
# Marker Hover
# - Arm the drone and takeoff using the Realsense in GUIDED mode.
# - Search for the sample zone while ascending to a search altitude
# - Approach the sample zone once detected and positional hover at the desired altitude
# - Lands if the sample zone was not detected.
#
# ---------------------------------------------------------------------------------- #

import time
import numpy as np
import transformations as tf

# Custom packages
from utils import dronekit_utils, file_utils
from simple_pid import PID
from slam import realsense_localization

HOVER_THRESHOLD = 0.1 # Meters; Max XY positional error allowed to begin descent to hover altitude
MAX_SEARCH_TIME = 200 # Seconds; How long to search for the marker before giving up
DETECTION_TIME = 3 # Seconds; How long the marker must be detected to be considered found
RUNNING_AVG_LENGTH = 10 # Higher will produce more smoothing at the cost of lag
DEFAULT_SEARCH_ALT = 2 # Meters

# Distance to the camera relative to the UAV in meters
# UAV coord. system is considered to be the x and y centers, and z is equal with the lid
BODY_TRANSLATION_X = 0.102
BODY_TRANSLATION_Y = -.0058
BODY_TRANSLATION_Z = 0.0242

# Pose transform from Aruco to UAV body coord. system
H_BODY_REF_CAM_REF_ARUCO = np.array([[0, -1, 0, BODY_TRANSLATION_X],
                                     [1,  0, 0, BODY_TRANSLATION_Y],
                                     [0,  0, 1, BODY_TRANSLATION_Z],
                                     [0,  0, 0, 1]])

# Pose transform from yellow marker to UAV body coord. system
H_BODY_REF_CAM_REF_YELLOW = np.array([[0, -1, 0, BODY_TRANSLATION_X],
                                      [-1, 0, 0, BODY_TRANSLATION_Y],
                                      [0,  0, 1, BODY_TRANSLATION_Z],
                                      [0,  0, 0, 1]])

# PID gains for sending navigation commands
PID_X = PID(0.35, 0, 0.005, setpoint=0)
PID_Y = PID(0.35, 0, 0.005, setpoint=0)
PID_Z = PID(0.35, 0, 0.005, setpoint=0) # Setpoint is set to hover altitude later

# Should be equal to or higher than frequency of your image processing algorithm
PID_X.sample_time = 0.01
PID_Y.sample_time = 0.01
PID_Z.sample_time = 0.01

VEHICLE_POSE_DIR = "marker_detection/logs/pose_data"
VEHICLE_POSE_BASE = "vehicle_pose"
VEHICLE_POSE_FILE = file_utils.create_file_name_chronological(VEHICLE_POSE_DIR, VEHICLE_POSE_BASE, "txt")

# Transform the marker pose into the UAV body frame (NED)
# marker_pose: list of x,y,z marker position
def marker_ref_to_body_ref(H_cam_ref_marker, marker_tracker, vehicle):

    # Select camera to body transform
    if marker_tracker.get_marker_type == "aruco":

        # Aruco marker
        H_body_ref_cam_ref = H_BODY_REF_CAM_REF_ARUCO
    else:

        # Yellow marker
        H_body_ref_cam_ref = H_BODY_REF_CAM_REF_YELLOW


    # Transform from body to NED coord. system, relative to the UAV origin
    roll = vehicle.attitude.roll
    pitch = vehicle.attitude.pitch

    # Z-component is removed, since depth is always a 2D calculation here
    roll_transform = np.array([[1,      0,             0,         0],
                         [0, np.cos(roll),  -np.sin(roll),  0],
                         [0,      0,             1,         0],
                         [0,      0,             0,        1]])

    pitch_transform = np.array([[np.cos(pitch),   0, np.sin(pitch),   0],
                          [      0,         1,       0,         0],
                          [-np.sin(pitch),  0, np.cos(pitch),   0],
                          [      0,         0,       0,         1]])

    # Transform to NED offset pose - AKA NED relative to UAV's current location
    # When using the Realsense, NED is equivalent to FRD (Forward Right Down)
    ned_offset_pose = np.matmul(roll_transform, np.matmul(pitch_transform,
                                                    np.matmul(H_body_ref_cam_ref, np.append(H_cam_ref_marker, 1))))


    return ned_offset_pose[:-1]

# Initial search for the marker over a long distance
# Ascend to desired search altitude
# Search for the marker for a set time period. If detection consistency is > 95%, return True
def marker_search_long_range(vehicle, marker_tracker, search_alt=DEFAULT_SEARCH_ALT):

    marker_list = [] # Keep track of potential markers, how many times they've been detected, and their pose

    start_time = time.time()

    # Ascend to search altitude
    # dronekit_utils.move_relative_alt(vehicle, search_alt)
    #
    # Wait until the vehicle reaches its search altitude
    # dronekit_utils.wait_for_nav_body_command(vehicle)

    # Search for the marker for a set amount of time
    while time.time() - start_time < MAX_SEARCH_TIME:

        # Track marker in image frame
        marker_tracker.track_marker_long_distance(alt=30)

        if marker_tracker.is_marker_found():

            # Get marker pose
            marker_pose = marker_tracker.get_pose()

            # Marker consistency check
            # When a new marker is detected, it is compared to all previously detected markers. If it closely matches
            # a previously detected marker, then it is counted as an additional detection of the previous marker.
            # This allows for outliers to be detected and ignored
            if marker_list:

                # Calculate the difference between current marker pose and all previous marker poses
                diff_list = [mpose[1] - marker_pose for mpose in marker_list]

                # Calculate the average differences between the markers
                diff_avg = [np.average(np.abs(diff)) for diff in diff_list]

                # Find the previous marker that most closely matches the current marker
                closest_match = np.amin(diff_avg)
                closest_match_idx = int(np.argmin(diff_avg))
                # TODO: Add 0,1 to 0s
                if np.all(np.abs(diff_list[closest_match_idx] / marker_list[closest_match_idx][1]) < 0.20):
                    marker_list[closest_match_idx][0] += 1
                else:

                    # First marker added to list
                    marker_list.append([1, marker_pose])
            else:
                marker_list.append([1, marker_pose])

    if marker_list:
        most_detected_marker = np.amax([marker_count[0] for marker_count in marker_list])
        most_detected_marker_idx = int(np.argmax([marker_count[0] for marker_count in marker_list]))

        if most_detected_marker > 5:
            return True, marker_list[most_detected_marker_idx]

    return False

def marker_search_close_range(vehicle, marker_tracker):
    pass

def marker_approach(vehicle, marker_tracker, marker_pose):

    # Move 80% of the way to the estimated marker position
    nav_command = 0.80 * marker_ref_to_body_ref(marker_pose, marker_tracker, vehicle)

    dronekit_utils.goto_position_target_body_offset_ned(vehicle,
                                                        forward=nav_command[0],
                                                        right=nav_command[1],
                                                        down=0)

    #
# Navigate towards the marker at the current altitude
def marker_center(vehicle, pose_avg):

    # Send position command to the vehicle
    dronekit_utils.goto_position_target_body_offset_ned(vehicle,
                                                        forward=pose_avg[0],
                                                        right=pose_avg[1],
                                                        down=0)

# Navigate towards the marker and the desired hovering altitude
def marker_alt_hover(vehicle, pose_avg):

    # Send position command to the vehicle
    dronekit_utils.goto_position_target_body_offset_ned(vehicle,
                                                        forward=pose_avg[0],
                                                        right=pose_avg[1],
                                                        down=pose_avg[2])

# Detect a marker and hover above it. The vehicle will remain still until a marker is detected. Then it will approach
# the marker at the current altitude until it is directly above the marker. Finally, the vehicle will ascend/descend
# to the desired hovering altitude. If the marker has not been detected for a while, the function will return False.
def marker_hover(vehicle, marker_tracker, rs=None, hover_alt=None, debug=0):

    if hover_alt is None:
        hover_alt = vehicle.location.global_relative_frame.alt

    # Set the PID setpoint to the desired hover altitude
    # Negative because of NED coord. system
    PID_Z.setpoint = -hover_alt

    # Queue to contain running average for Aruco markers
    pose_queue = np.zeros((RUNNING_AVG_LENGTH, 3), dtype=float)
    count = 0

    if debug > 0:
        vehicle_pose_file = file_utils.open_file(VEHICLE_POSE_FILE)

    # Hover until manual override
    start_time = time.time()
    time_found = time.time()
    print("Tracking marker...")

    while dronekit_utils.is_guided(vehicle):

        # Track marker
        marker_tracker.track_marker(alt=vehicle.location.global_relative_frame.alt)

        if marker_tracker.is_marker_found():

            # Note the most recent time marker was found
            time_found = time.time()

            # Get the marker pose relative to the camera origin
            marker_pose = marker_tracker.get_pose()

            # Transform the marker pose into UAV body frame (NED)
            marker_pose_body_ref = marker_ref_to_body_ref(marker_pose, marker_tracker, vehicle)

            # PID control; input is the UAV pose relative to the marker (hence the negatives)
            control_pose = np.array([[PID_X(-marker_pose_body_ref[0]),
                                      PID_Y(-marker_pose_body_ref[1]),
                                      PID_Z(-marker_pose_body_ref[2])]])

            # Keep a running average if using Aruco markers
            # Yellow marker processing is too slow to use a running average
            if marker_tracker.get_marker_type() == "aruco":

                pose_queue[count % len(pose_queue)] = control_pose
                count += 1

                # Take the average
                if count < RUNNING_AVG_LENGTH - 1:
                   pose_avg = np.sum(pose_queue, axis=0) / count
                else:
                   pose_avg = np.average(pose_queue, axis=0)
            else:

                # Yellow marker
                pose_avg = control_pose[0]

            # Approach the marker at the current altitude until above the marker, then navigate to the hover altitude.
            if np.abs(pose_avg[0]) < HOVER_THRESHOLD and np.abs(pose_avg[1]) < HOVER_THRESHOLD:
                print("Marker Hover")
                marker_alt_hover(vehicle, pose_avg)
            else:
                print("Approaching Marker")
                marker_center(vehicle, pose_avg)

            if debug > 0 and rs is not None:
                data = rs.read()
                rs_pose = tf.quaternion_matrix([data.rotation.w,
                                                     data.rotation.x,
                                                     data.rotation.y,
                                                     data.rotation.z])

                rs_pose[0][3] = data.translation.x
                rs_pose[1][3] = data.translation.y
                rs_pose[2][3] = data.translation.z
                vehicle_pose = realsense_localization.rs_to_body(rs_pose)
                vehicle_trans = np.array(tf.translation_from_matrix(vehicle_pose))

                vehicle_pose_file.write("{} {} {} {} 0 0 0 0\n".format(time.time() - start_time,
                                                                     vehicle_trans[0],
                                                                     vehicle_trans[1],
                                                                     vehicle_trans[2]))

            if debug > 2:
               print("Nav command: {} {} {}".format(pose_avg[0], pose_avg[1], pose_avg[2]))
        else:
            if time.time() - time_found > 5:
                return False

        # Maintain frequency of sending commands
        marker_tracker.wait()