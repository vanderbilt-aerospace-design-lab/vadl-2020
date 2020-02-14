''' Flight landing script
Goal is to takeoff, fly to a set altitude,
switch to tracking mode, and maintain a
hover over the marker. Manual override always available by switching out of GUIDED mode. '''

import os
import time
import argparse
import numpy as np
import transformations as tf
from dronekit import VehicleMode
from apscheduler.schedulers.background import BackgroundScheduler

# Custom packages
from utils import dronekit_utils, file_utils
from simple_pid import PID
from slam_evaluation import realsense_localization
from marker_detection.marker_tracker import ArucoTracker, ColorMarkerTracker
from marker_detection.camera import Realsense

TARGET_ALTITUDE = 1 # Meters
HOVER_ALTITUDE = 1 # Meters
DEFAULT_FREQ = 20 # Hz
DEFAULT_MARKER = "aruco"

VEHICLE_POSE_DIR = "marker_detection/pose_data"
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
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]])
    else:
        # Yellow marker
        body_transform = np.array([[0, -1, 0, 0],
                                   [-1, 0, 0, 0],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]])

    # Transform to body pose
    body_pose = np.matmul(np.append(marker_pose, 1), body_transform)

    return body_pose[:-1]

def marker_hover(vehicle, marker_tracker, rs=None):

    pid_x = PID(1, 0, 0, setpoint=0)
    pid_y = PID(1, 0, 0, setpoint=0)
    pid_z = PID(1, 0, 0, setpoint=-0.5)

    pid_x.sample_time = 0.01
    pid_y.sample_time = 0.01
    pid_z.sample_time = 0.01

    # Hover until manual override
    x_queue = []
    y_queue = []
    z_queue = []

    if args["debug"] > 0:
        vehicle_pose_file = file_utils.open_file(VEHICLE_POSE_FILE)

    start_time = time.time()
    print("Tracking marker...")
    while True:
    # while vehicle.mode == VehicleMode("GUIDED"):

        # Track marker
        marker_tracker.track_marker(alt=vehicle.location.global_relative_frame.alt)

        if marker_tracker.is_marker_found():

            marker_pose = marker_tracker.get_pose()

            # Transform pose to UAV body frame; proportional gain
            marker_pose_body_ref = marker_ref_to_body_ref(marker_pose)

            # Pid response, input is UAV rel. to marker
            x = pid_x(-marker_pose_body_ref[0])
            y = pid_y(-marker_pose_body_ref[1])
            z = pid_z(-marker_pose_body_ref[2])

            x_queue.append(x)
            y_queue.append(y)
            z_queue.append(z)

            if len(x_queue) > 10:
                x_queue.pop(0)
                y_queue.pop(0)
                z_queue.pop(0)

            x_avg = np.average(x_queue)
            y_avg = np.average(y_queue)
            z_avg = np.average(z_queue)

            # Send position command to the vehicle
            dronekit_utils.goto_position_target_body_offset_ned(vehicle,
                                                                forward=x_avg,
                                                                right=y_avg,
                                                                down= -HOVER_ALTITUDE - z_avg)


            if args["debug"] > 0 and rs is not None:
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

            if args["debug"] > 2:
               print("Nav command: {} {} {}".format(x_avg, y_avg, z_avg))

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

    rs = Realsense()

    # Create a scheduler to send Mavlink commands in the background
    # sched = BackgroundScheduler()

    # Begin realsense localization in the background
    # realsense_localization.start(vehicle, rs=rs, scheduler=sched)

    # Arm the UAV
    # dronekit_utils.arm_realsense_mode(vehicle)

    # Takeoff and fly to a target altitude
    # dronekit_utils.takeoff(vehicle, TARGET_ALTITUDE)

    # Give time to stabilize in flight
    # time.sleep(5)

    # Set the UAV speed
    # vehicle.airspeed = 0.10

    # Maintain hover over a marker
    marker_hover(vehicle, marker_tracker, rs)

if __name__ == "__main__":
    main()
