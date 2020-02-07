''' Flight landing script
Goal is to takeoff, fly to a set altitude,
switch to tracking mode, and maintain a
hover over the marker. Manual override always available. '''

from dronekit import VehicleMode
import argparse
import numpy as np
from simple_pid import PID
import os
import time
from utils import dronekit_utils, file_utils
from marker_detection.marker_tracker import ArucoTracker
from slam_evaluation import realsense_localization

TARGET_ALTITUDE = 1 # Meters

# Files relative to project directory
VIDEO_FILE_DIR = "marker_detection/videos"
PID_DIR = "marker_detection/pid_data/"
file_utils.make_dir(PID_DIR)
VIDEO_FILE_SAVE = VIDEO_FILE_DIR + file_utils.create_file_name_date() + ".mp4"
PID_FILE = PID_DIR + file_utils.create_file_name_date() + ".txt"

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Fly a UAV to a set altitude and hover over a marker.')
parser.add_argument('--sitl',
                   help="Vehicle connection target string. If specified, SITL will be used.")
parser.add_argument('-v','--video', default=0,
                    help="Play video instead of live stream.")
parser.add_argument("-p", "--picamera", type=int, default=-1,
 	help="Indicates whether or not the Raspberry Pi camera should be used")
parser.add_argument('-d',"--debug", default=0,
                   help="Whether or not videos should be saved and print statements should be used.")
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
                         "just the file name w/ or w/o .txt at the end. If none specified, defaults to the current date.")

args = vars(parser.parse_args())

if not isinstance(args["video"], int):
    if not os.path.exists(args["video"]):
        raise Exception("ERROR: Video file does not exist")
    VIDEO_FILE_STREAM = args["video"]
else:
    VIDEO_FILE_STREAM = 0

if not isinstance(args["video"], int):
    if not os.path.exists(args["video"]):
        raise Exception("ERROR: Video file does not exist")
    VIDEO_FILE_STREAM = args["video"]
else:
    VIDEO_FILE_STREAM = 0

def aruco_ref_to_body_ref(aruco_pose):
    # Forward facing
    aruco_to_body_transform = np.array([[-1, 0, 0, 0],
                                        [0, -1, 0, 0],
                                        [0, 0, -1, 0],
                                        [0, 0, 0, -1]])

    # Original
    aruco_pose = np.array([aruco_pose[0], aruco_pose[1], aruco_pose[2], 1])

    body_pose = np.matmul(aruco_pose, aruco_to_body_transform)

    return np.array([body_pose[0], body_pose[1], body_pose[2]])

def marker_hover(vehicle, marker_tracker):

    pid = PID(1, 0.1, 0.05, setpoint=0)
    pid.sample_time = 0.01

    if args["debug"]:
        # Open text file to store UAV position
        pid_file = file_utils.open_file(PID_FILE)

    # Hover until manual override
    print("Tracking marker...")
    while vehicle.mode == VehicleMode("GUIDED"):
        # Track marker
        marker_tracker.track_marker(alt=vehicle.location.global_relative_frame.alt)

        if marker_tracker.is_marker_found():

            # Flip signs because aruco has bottom right and down as positive axis
            marker_pose_cam_ref = marker_tracker.get_pose()

            '''Aruco Marker'''
            marker_pose_body_ref = aruco_ref_to_body_ref(marker_pose_cam_ref)

            command_forward = marker_pose_body_ref[0]
            command_right = marker_pose_body_ref[1]

            print(marker_pose_body_ref)

            # Send position command to the vehicle
            dronekit_utils.goto_position_target_body_offset_ned(vehicle,
                                                                forward=command_forward,
                                                                right=command_right,
                                                                down=0)
            # print(vehicle.location.local_frame)

            # Mess w/ this during test
            time.sleep(0.5)

            if args["debug"]:
                # print("Sending: {}, {}".format(command_right, command_forward))
                pid_file.write("{} {}\n".format(command_forward, command_right))
        else:
            if args["debug"]:
                pid_file.write("{} {}\n".format("N/A", "N/A"))

def main():
    # Create Marker Detector; before UAV takes off because takes a while to process
    marker_tracker = ArucoTracker(src=args["video"],
                                use_pi=args["picamera"],
                                debug=args["debug"],
                                resolution=args["resolution"],
                                framerate=args["fps"],
                                video_dir=args["dir"],
                                video_file=args["name"],
                                pose_file=args["pose_file"])

    # Connect to the Pixhawk
    vehicle = dronekit_utils.connect_vehicle_args(args)

    # Begin realsense localization in the background
    realsense_localization.start(vehicle)

    # Wait for home location to be set
    while vehicle.home_location is None:
        print("Waiting for home location")
        time.sleep(1)
    print(vehicle.home_location)
    #dronekit_utils.wait_for_home_location(vehicle)

    # Arm the UAV
    dronekit_utils.arm_realsense_mode(vehicle)

    # Takeoff and fly to a target altitude
#    dronekit_utils.takeoff(vehicle, TARGET_ALTITUDE)

    # Maintain hover over a marker
 #   marker_hover(vehicle, marker_tracker)

    # Land the UAV (imprecisely)
  #  dronekit_utils.land(vehicle)


if __name__ == "__main__":
    main()
