'''Test for integrated sampling zone mission sequence:
UAV will takeoff, locate sampling zone (intended to be in range), center on it, hover at 3ft, and sample'''

from dronekit import VehicleMode
import argparse
import os
import time
from utils import dronekit_utils, file_utils
from marker_detection.marker_tracker import YellowMarkerTracker, ArucoTracker
from marker_detection.marker_hover import marker_hover, marker_search
from feather_com.pi import manual_only

HOVER_ALTITUDE = 0.914 # Meters
TAKEOFF_ALTITUDE = 1.5 # Meters
LAND_ALTITUDE = 0.35 # Meters

# Files relative to project directory
VIDEO_FILE_DIR = "marker_detection/videos"
PID_DIR = "marker_detection/pid_data/"
file_utils.make_dir(PID_DIR)
VIDEO_FILE_SAVE = VIDEO_FILE_DIR + file_utils.create_file_name_date() + ".mp4"
PID_FILE = PID_DIR + file_utils.create_file_name_date() + ".txt"

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Perform a scaled sample mission.')
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
parser.add_argument('--marker_length', default=None,
                    help="yellow marker side length")

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

def main():
    # Create Marker Detector; before UAV takes off because takes a while to process
    marker_tracker = YellowMarkerTracker(src=args["video"],
                                        use_pi=args["picamera"],
                                        debug=args["debug"],
                                        resolution=args["resolution"],
                                        framerate=args["fps"],
                                        video_dir=args["dir"],
                                        video_file=args["name"],
                                        pose_file=args["pose_file"],
                                        marker_length=args["marker_length"])

    marker_tracker2 = ArucoTracker(src=args["video"],
                                   use_pi=args["picamera"],
                                   debug=args["debug"],
                                   resolution=args["resolution"],
                                   framerate=args["fps"],
                                   video_dir=args["dir"],
                                   video_file=args["name"],
                                   pose_file=args["pose_file"])

    # Connect to the Pixhawk
    vehicle = dronekit_utils.connect_vehicle_args(args)

    # Arm the UAV
    dronekit_utils.arm(vehicle)

    # Takeoff and fly to a target altitude
    dronekit_utils.takeoff(vehicle, TAKEOFF_ALTITUDE)

    # Search for marker
    print("Searching...")
    marker_found = marker_search(vehicle, marker_tracker, search_alt=TAKEOFF_ALTITUDE)

    # If marker is not found with marker_search timeout above, then UAV will pass this sampling loop and land
    # If marker is found, UAV will begin to hover over the marker and then perform sampling
    # If marker_hover is called without run_time, it will run until z target error is less than 0.1m, then exit
    print("Centering...")
    if marker_found:
        # Approach desired sampling height
        marker_hover(vehicle, marker_tracker, hover_alt=HOVER_ALTITUDE)

        # Perform air-based sampling
        print("Sampling...")
        manual_only.lower_tool()
        marker_hover(vehicle, marker_tracker, hover_alt=HOVER_ALTITUDE, run_time=7)
        manual_only.idle()

        manual_only.close_tool()
        marker_hover(vehicle, marker_tracker, hover_alt=HOVER_ALTITUDE, run_time=2)
        manual_only.raise_tool()
        marker_hover(vehicle, marker_tracker, hover_alt=HOVER_ALTITUDE, run_time=8.25)
        manual_only.idle()

        # Navigate to Aruco - needs to be in sight from 3ft or we're sunk
        print("Leaving...")
        marker_hover(vehicle, marker_tracker2, hover_alt=LAND_ALTITUDE)

    # Land the UAV (imprecisely)
    print("Landing...")
    dronekit_utils.land(vehicle)


if __name__ == "__main__":
    main()
