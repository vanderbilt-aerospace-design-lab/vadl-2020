''' Sample zone approach and hover

UAV will takeoff to a set altitude, search for the sample zone, navigate to it, and descend to the desired
 sampling altitude'''

import os
import argparse
from apscheduler.schedulers.background import BackgroundScheduler

# Custom packages
from utils import dronekit_utils, file_utils
from slam import realsense_localization
from marker_detection.marker_tracker import ArucoTracker, YellowMarkerTracker
from marker_detection.camera import Realsense
from marker_detection.marker_nav_utils import *

TAKEOFF_ALTITUDE = 0.5 # Meters
SEARCH_ALTITUDE = 0.7 # Meters
HOVER_ALTITUDE = 2 * 0.3048 # Meters
DEFAULT_FREQ = 2 # Hz
MARKER_LENGTH = 0.24

VEHICLE_POSE_DIR = "marker_detection/logs/pose_data"
VEHICLE_POSE_BASE = "vehicle_pose"
VEHICLE_POSE_FILE = file_utils.create_file_name_chronological(VEHICLE_POSE_DIR, VEHICLE_POSE_BASE, "txt")

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Fly a UAV to a set altitude and hover over a marker.')
parser.add_argument('--sitl',
                    help="Vehicle connection target string. If specified, SITL will be used.")
parser.add_argument('-v','--video', default=0,
                    help="Play video instead of live stream.")
parser.add_argument("-p", "--pi", type=int, default=1,
 	                help="Indicates whether or not the Raspberry Pi camera should be used. Defaults to Pi.")
parser.add_argument('-d',"--debug", default=0, type=int,
                    help="Whether or not videos and pose files should be saved and print statements are used."
                         "1 - save pose files, 2 - save pose files and videos, "
                         "3 - save pose files and videos and print statements")
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
parser.add_argument('--frequency', default=DEFAULT_FREQ,
                    help="Frequency of marker tracking")

args = vars(parser.parse_args())

if not isinstance(args["video"], int):
    if not os.path.exists(args["video"]):
        raise Exception("ERROR: Video file does not exist")
    VIDEO_FILE_STREAM = args["video"]
else:
    VIDEO_FILE_STREAM = 0

def main():

    marker_tracker = YellowMarkerTracker(src=args["video"],
                                        use_pi=args["pi"],
                                        resolution=args["resolution"],
                                        framerate=args["fps"],
                                        marker_length=MARKER_LENGTH,
                                        freq=args["frequency"],
                                        debug=args["debug"],
                                        video_dir=args["dir"],
                                        video_file=args["name"],
                                        pose_file=args["pose_file"])

    # Connect to the Pixhawk
    vehicle = dronekit_utils.connect_vehicle_args(args)

    rs = Realsense()

    # Create a scheduler to send Mavlink commands in the background
    sched = BackgroundScheduler()

    # Begin realsense localization in the background
    realsense_localization.start(vehicle, rs=rs, scheduler=sched)

    time.sleep(10)

    # Arm the UAV
    dronekit_utils.arm_realsense_mode(vehicle)

    # Takeoff and fly to a target altitude
    dronekit_utils.takeoff(vehicle, TAKEOFF_ALTITUDE)

    # Give time to stabilize in flight
    time.sleep(2)

    # Set the UAV speed
    vehicle.airspeed = 5

    # Search for sample zone
    print("Initial Marker Search")
    marker_found, marker_pose = marker_search_long_range(vehicle, marker_tracker, SEARCH_ALTITUDE)

    if marker_found:

        print("Approaching Marker")
        marker_found = marker_approach(vehicle, marker_tracker, marker_pose)

        if marker_found:
            marker_hover(vehicle, marker_tracker, rs, HOVER_ALTITUDE)

    # Land the UAV
    dronekit_utils.land(vehicle)

if __name__ == "__main__":
    main()
