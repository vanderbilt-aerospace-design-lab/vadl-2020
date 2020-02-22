# ---------------------------------------------------------------------------------- #
#
# Marker Navigation Utils
# - Initial Marker Search: Initial marker detection from afar
# - Marker Approach: Take the initial marker pose estimate and quickly fly to the marker's vicinity
# - Marker Hover: Maintain a positional hover above a marker
#
# ---------------------------------------------------------------------------------- #

import time
import numpy as np

# Custom packages
from utils import dronekit_utils, file_utils
from simple_pid import PID

## Constants ##

# Initial marker search
DEFAULT_SEARCH_ALT = 2 # Meters; Altitude UAV flies to perform initial search
MAX_SEARCH_TIME_LONG_RANGE = 25 # Seconds; How long to search for the marker during long range search
MAX_SEARCH_TIME_CLOSE_RANGE = 10 # Seconds; How long to search for the marker during close range search
POSE_DIFFERENCE_THRESHOLD = 0.1 # Maximum % difference allowed between newest marker and previously detected markers
MIN_DETECTION_COUNT = 5 # Minimum number of detections during search time required to satisfy marker detection
DETECTION_THRESHOLD_CLOSE_RANGE = 0.90 # Percentage of frames that must detect the marker in close range search
GROUNDSPEED_MARKER_APPROACH = 5 # M/s; Flight speed during marker approach

# Marker hover
HOVER_THRESHOLD = 0.1 # Meters; Max XY positional error allowed before UAV begins descent to hover altitude
RUNNING_AVG_LENGTH = 10 # How many pose estimates over time to average; Higher will produce more smoothing and lag
FAST_SPEED = 5 # M/s; Travel speed when far from the marker
MED_SPEED = 1 # M/s; Travel speed when medium far
HOVER_SPEED = 0.1 # M/s; Travel speed when maintaining hover above marker
FAR_DIST = 10 * 0.3048 # Meters; Distance considered "far" during marker hover
MED_DIST = 1 * 0.3048 # Meters; Distance considered "medium" during marker hover
MAX_LOST_DETECTION_TIME = 5 # Seconds; Maximum time the marker can be lost before function ends

## Pose Transformations

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

## PID
PID_X = PID(0.35, 0, 0.005, setpoint=0)
PID_Y = PID(0.35, 0, 0.005, setpoint=0)
PID_Z = PID(0.35, 0, 0.005, setpoint=0) # Setpoint is set to hover altitude later

# Should be equal to or higher than frequency of your image processing algorithm
PID_X.sample_time = 0.01
PID_Y.sample_time = 0.01
PID_Z.sample_time = 0.01

## Debug files
VEHICLE_POSE_DIR = "marker_detection/logs/pose_data"
VEHICLE_POSE_BASE = "vehicle_pose"
VEHICLE_POSE_FILE = file_utils.create_file_name_chronological(VEHICLE_POSE_DIR, VEHICLE_POSE_BASE, "txt")


# Transform the marker pose from the camera frame to the UAV NED offset frame
# Marker pose is received relative to the camera center.
# 1) Transform from camera frame to body frame
# 2) Transform from body frame to NED offset frame
# Body frame means axes tilt with the UAV. The body frame can be represented in the NED offset frame by standard
# roll, pitch, and yaw angles. The "offset" means it is relative to the UAV, not the takeoff origin.
# marker_pose: list of x,y,z marker position
def marker_ref_to_body_ref(H_cam_ref_marker, marker_tracker, vehicle):

    # Select the marker transform
    if marker_tracker.get_marker_type == "aruco":

        # Aruco marker
        H_body_ref_cam_ref = H_BODY_REF_CAM_REF_ARUCO
    else:

        # Yellow marker
        H_body_ref_cam_ref = H_BODY_REF_CAM_REF_YELLOW


    # Transform from body to NED offset coord. system, relative to the UAV
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

    # Transform to NED offset pose
    # When using the Realsense, NED is equivalent to FRD (Forward Right Down)
    ned_offset_pose = np.matmul(roll_transform, np.matmul(pitch_transform,
                                                    np.matmul(H_body_ref_cam_ref, np.append(H_cam_ref_marker, 1))))


    return ned_offset_pose[:-1]


# Initial search for the marker over a long distance
# 1) Ascend to desired search altitude
# 2) Search for the marker for a set time period. Compare each new marker potential to all previously detected markers.
#    If a single marker is frequently detected over others, then return that marker's pose
def marker_search_long_range(vehicle, marker_tracker, search_alt=DEFAULT_SEARCH_ALT):

    marker_list = [] # Keep track of potential markers, how many times they've been detected, and their pose

    # Ascend to search altitude
    # dronekit_utils.move_relative_alt(vehicle, search_alt)

    # Wait until the vehicle reaches its search altitude
    # dronekit_utils.wait_for_nav_body_command(vehicle)

    start_time = time.time()

    # Search for the marker for a set time period
    while time.time() - start_time < MAX_SEARCH_TIME_LONG_RANGE:

        # Track marker
        # marker_tracker.track_marker_long_distance(alt=vehicle.location.global_relative_frame.alt)
        marker_tracker.track_marker_long_distance(alt=30)

        if marker_tracker.is_marker_found():

            # Get marker pose
            marker_pose = marker_tracker.get_pose()

            # Marker consistency check
            # When a new marker is detected, it is compared to all previously detected markers. If it closely matches
            # a previously detected marker, then it is counted as an additional detection of the previous marker.
            # This allows the frequency of marker detection to be considered
            if marker_list:

                # Calculate the difference between current marker pose and all previous marker poses
                diff_list = [mpose[1] - marker_pose for mpose in marker_list]

                # Calculate the magnitude of the difference vector between the markers
                diff_hyp = [np.sqrt(np.sum(np.square(diff))) for diff in diff_list]

                # Find the previous marker that most closely matches the current marker
                closest_match_idx = int(np.argmin(diff_hyp))

                # Increment marker count if they closely match
                if np.all(np.abs(diff_list[closest_match_idx] / marker_list[closest_match_idx][1]) < POSE_DIFFERENCE_THRESHOLD):
                    marker_list[closest_match_idx][0] += 1

                    # Average the previous and newest marker poses
                    marker_list[closest_match_idx][1] = np.average([marker_list[closest_match_idx][1], marker_pose],
                                                                   axis=0)
            else:

                # First marker added to list
                marker_list.append([1, marker_pose])

    if marker_list:

        # Find the marker that was detected the most
        most_detected_marker = np.amax([marker_count[0] for marker_count in marker_list])
        most_detected_marker_idx = int(np.argmax([marker_count[0] for marker_count in marker_list]))

        # Must pass detection threshold
        if most_detected_marker > MIN_DETECTION_COUNT:
            return True, marker_list[most_detected_marker_idx] # Marker found, marker pose

    return False, 0


# Marker search over a close range
# Simplified version of long range search that performs marker detection for a set time period and checks percentage
# of frames the marker was detected in. Must pass the detection criteria to be sure it is a real marker.
def marker_search_close_range(vehicle, marker_tracker):

    total_count = 0 # Num. frames processed
    detected_count = 0 # Num. frames w/ a detected marker

    start_time = time.time()

    # Search for the marker for a set time period
    while time.time() - start_time < MAX_SEARCH_TIME_CLOSE_RANGE:

        # Track marker
        # marker_tracker.track_marker_long_distance(alt=vehicle.location.global_relative_frame.alt)
        marker_tracker.track_marker_long_distance(alt=30)

        if marker_tracker.is_marker_found():
            detected_count += 1

        total_count += 1.0

    return detected_count / total_count > DETECTION_THRESHOLD_CLOSE_RANGE


# Approach the marker quickly from a long distance
# After initial long range search is completed, the UAV will navigate to the vicinity of the detected marker.
# Once reached, close range marker search will be performed to confirm the marker is nearby.
def marker_approach(vehicle, marker_tracker, marker_pose):

    vehicle.groundspeed = GROUNDSPEED_MARKER_APPROACH

    # Move 80% of the way to the estimated marker position
    nav_command = 0.80 * marker_ref_to_body_ref(marker_pose, marker_tracker, vehicle)

    dronekit_utils.goto_position_target_body_offset_ned(vehicle,
                                                        forward=nav_command[0],
                                                        right=nav_command[1],
                                                        down=0)

    # Wait for navigation to finish
    dronekit_utils.wait_for_nav_body_command(vehicle)

    # Verify the marker is nearby
    marker_found = marker_search_close_range(vehicle, marker_tracker)

    return marker_found


# Set the UAV groundspeed based on how far away it is from the marker
def set_groundspeed(vehicle, marker_pose_body_ref):

    if np.abs(marker_pose_body_ref[0]) > FAR_DIST or np.abs(marker_pose_body_ref[1]) > FAR_DIST:
        vehicle.groundspeed = FAST_SPEED
    elif np.abs(marker_pose_body_ref[0]) > MED_DIST or np.abs(marker_pose_body_ref[1]) > MED_DIST:
        vehicle.groundspeed = MED_SPEED
    else:
        vehicle.groundspeed = HOVER_SPEED


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
# to the desired hovering altitude. If the marker has not been detected for a while, the function will return.
def marker_hover(vehicle, marker_tracker, hover_alt=None, debug=0):

    # Hover altitude is current altitude if none specified
    if hover_alt is None:
        hover_alt = vehicle.location.global_relative_frame.alt

    # Set the PID Z setpoint to the desired hover altitude
    # Negative because of NED coord. system
    PID_Z.setpoint = -hover_alt

    # Queue to contain running average for Aruco markers
    pose_queue = np.zeros((RUNNING_AVG_LENGTH, 3), dtype=float)
    count = 0

    # Hover until manual override
    time_found = time.time()
    print("Tracking marker...")

    while dronekit_utils.is_guided(vehicle):

        # Track marker
        # marker_tracker.track_marker(alt=vehicle.location.global_relative_frame.alt)
        marker_tracker.track_marker_long_distance(alt=30)

        if marker_tracker.is_marker_found():

            # Note the most recent time marker was found
            time_found = time.time()

            # Get the marker pose relative to the camera origin
            marker_pose = marker_tracker.get_pose()

            # Transform the marker pose into UAV body frame (NED)
            marker_pose_body_ref = marker_ref_to_body_ref(marker_pose, marker_tracker, vehicle)

            # Set the groundspeed based on marker position
            set_groundspeed(vehicle, marker_pose_body_ref)

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
                marker_alt_hover(vehicle, pose_avg)
            else:
                marker_center(vehicle, pose_avg)

            if debug > 2:
               print("Nav command: {} {} {}".format(pose_avg[0], pose_avg[1], pose_avg[2]))

        else:

            # End if marker is not detected for a while
            if time.time() - time_found > MAX_LOST_DETECTION_TIME:
                return False

        # Maintain frequency of sending commands
        marker_tracker.wait()