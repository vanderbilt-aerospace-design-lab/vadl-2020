''' Flight landing script
Goal is to takeoff, fly to a set altitude,
switch to tracking mode, and maintain a
hover over the marker. Manual override always available by switching out of GUIDED mode. '''

import os
import time
import argparse
import numpy as np
from dronekit import VehicleMode
from apscheduler.schedulers.background import BackgroundScheduler

# Custom packages
from utils import dronekit_utils
from slam_evaluation import realsense_localization
from marker_detection.marker_tracker import ArucoTracker, ColorMarkerTracker

TARGET_ALTITUDE = 1 # Meters
DEFAULT_FREQ = 20 # Hz
DEFAULT_MARKER = "aruco"

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Fly a UAV to a set altitude and hover over a marker.')
parser.add_argument('--sitl',
                    help="Vehicle connection target string. If specified, SITL will be used.")
parser.add_argument('-v','--video', default=0,
                    help="Play video instead of live stream.")
parser.add_argument("-p", "--pi", type=int, default=1,
 	                help="Indicates whether or not the Raspberry Pi camera should be used. Defaults to Pi.")
parser.add_argument('-d',"--debug", default=0,
                    help="Whether or not videos and pose files should be saved and print statements are used.")
parser.add_argument('-r','--resolution', type=int, default=480,
                    help="Camera resolution")
parser.add_argument('-f','--fps', type=int, default=30,
                    help="Camera frame rate")
parser.add_argument('--dir', default=None,
                    help="Directory to save file. Defaults to marker_detection/videos")
parser.add_argument('-n','--name', default=None,
                    help="File name. If none specified, defaults to the current date.")
parser.add_argument('--pose_file', default=None,
                    help="Pose file name for debugging. Do not input directory, "
                         "include .txt at the end. If no file specified, defaults to the current date.")
parser.add_argument('-m', '--marker', default=DEFAULT_MARKER,
                    help="Type of marker to track. 'aruco' or 'yellow'")
parser.add_argument('--frequency', default=DEFAULT_FREQ,
                    help="Frequency of marker tracking")

args = vars(parser.parse_args())

if not isinstance(args["video"], int):
    if not os.path.exists(args["video"]):
        raise Exception("ERROR: Video file does not exist")
    VIDEO_FILE_STREAM = args["video"]
else:
    VIDEO_FILE_STREAM = 0

# Transform the marker pose into the UAV body frame
# marker_pose: list of x,y,z marker position
def marker_ref_to_body_ref(marker_pose):
    if args["marker"] == "aruco":
        # Aruco marker
        body_transform = np.array([[0, -1, 0, 0],
                                   [1, 0, 0, 0],
                                   [0, 0, -1, 0],
                                   [0, 0, 0, 1]])
    else:
        # Yellow marker
        body_transform = np.array([[0, -1, 0, 0],
                                   [-1, 0, 0, 0],
                                   [0, 0, -1, 0],
                                   [0, 0, 0, 1]])

    # Transform to body pose
    body_pose = np.matmul(np.append(marker_pose, 1), body_transform)

    return body_pose[:-1]

def marker_hover(vehicle, marker_tracker):

    # Hover until manual override
    print("Tracking marker...")
    while vehicle.mode == VehicleMode("GUIDED"):

        # Track marker
        marker_tracker.track_marker(alt=vehicle.location.global_relative_frame.alt)

        if marker_tracker.is_marker_found():

            marker_pose = marker_tracker.get_pose()

            # Transform pose to UAV body frame
            marker_pose_body_ref = 0.5 * marker_ref_to_body_ref(marker_pose)

            # Send position command to the vehicle
            dronekit_utils.goto_position_target_body_offset_ned(vehicle,
                                                                forward=marker_pose_body_ref[0],
                                                                right=marker_pose_body_ref[1],
                                                                down=-1 - marker_pose_body_ref[2])

            #if args["debug"]:
             #   print("Nav Command: {}".format(marker_pose_body_ref))

        # Maintain frequency of sending commands
        marker_tracker.wait()

def main():

    # Create marker tracker
    if args["marker"] == "aruco":
        marker_tracker = ArucoTracker(src=args["video"],
                                      use_pi=args["pi"],
                                      debug=args["debug"],
                                      resolution=args["resolution"],
                                      framerate=args["fps"],
                                      freq=args["frequency"],
                                      video_dir=args["dir"],
                                      video_file=args["name"],
                                      pose_file=args["pose_file"])
    else:
        marker_tracker = ColorMarkerTracker(src=args["video"],
                                            use_pi=args["pi"],
                                            resolution=args["resolution"],
                                            framerate=args["fps"],
                                            freq=args["frequency"],
                                            debug=args["debug"],
                                            video_dir=args["dir"],
                                            video_file=args["name"],
                                            pose_file=args["pose_file"])

    # Connect to the Pixhawk
    vehicle = dronekit_utils.connect_vehicle_args(args)

    # Create a scheduler to send Mavlink commands in the background
    sched = BackgroundScheduler()

    # Begin realsense localization in the background
    realsense_localization.start(vehicle, sched)

    # Arm the UAV
    dronekit_utils.arm_realsense_mode(vehicle)

    # Takeoff and fly to a target altitude
    dronekit_utils.takeoff(vehicle, TARGET_ALTITUDE)

    # Give time to stabilize in flight
    time.sleep(5)

    # Set the UAV speed
    vehicle.airspeed = 0.10

    # Maintain hover over a marker
    marker_hover(vehicle, marker_tracker)

if __name__ == "__main__":
    main()
