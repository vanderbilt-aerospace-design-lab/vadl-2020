''' Flight landing script
    Goal is to takeoff, fly to a set altitude,
    switch to tracking mode, and maintain a
    hover over the marker. Manual override always available. '''

from dronekit import VehicleMode
import argparse
import numpy as np
from simple_pid import PID
import os

from utils import dronekit_utils
from marker_tracker import ArucoTracker

TARGET_ALTITUDE = 2 # Meters

# Files relative to project directory
VIDEO_FILE_SAVE = 'marker_detection/videos/marker_hover_1.mp4'
POSE_FILE = "marker_detection/pose_data/marker_pose_0.txt"
PID_FILE = "marker_detection/pid_data/pid_0.txt"

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Fly a UAV to a set altitude and hover over a marker.')
parser.add_argument('--sitl',
                   help="Vehicle connection target string. If specified, SITL will be used.")
parser.add_argument('-v','--video', default=0,
                    help="Play video instead of live stream.")
parser.add_argument("-p", "--picamera", type=int, default=-1,
 	help="Indicates whether or not the Raspberry Pi camera should be used")
parser.add_argument('-d',"--debug", default=0,
                   help="Vehicle connection target string. If specified, SITL will be used.")

args = vars(parser.parse_args())

if not isinstance(args["video"], int):
    if not os.path.exists(args["video"]):
        raise Exception("ERROR: Video file does not exist")
    VIDEO_FILE_STREAM = args["video"]
else:
    VIDEO_FILE_STREAM = 0

def connect_vehicle():
    #Start SITL if connection string specified
    if args["sitl"]:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        CONNECTION_STRING = sitl.connection_string()
    else:
        CONNECTION_STRING = "/dev/ttyAMA0"

    # Connect to the Vehicle
    return dronekit_utils.connect_vehicle(CONNECTION_STRING)

def marker_hover(vehicle, marker_tracker=ArucoTracker()):

    pid = PID(1, 0.1, 0.05, setpoint=0)
    pid.sample_time = 0.01

    if DEBUG:
        # Open text file to store UAV position
        pose_file = open(POSE_FILE, "w")
        pid_file = open(PID_FILE, "w")

    # Hover until manual override
    print("Tracking marker...")
    while vehicle.mode == VehicleMode("GUIDED"):

        # Track marker
        marker_tracker.track_marker(alt=vehicle.location.global_relative_frame.alt)

        # print("Vehicle: {}, {}".format(vehicle.location.local_frame.north, vehicle.location.local_frame.east))
        if marker_tracker.is_marker_found():

            # Flip signs because aruco has bottom right and down as positive axis
            marker_pose_aruco_ref = marker_tracker.get_pose()

            # Convert to UAV body reference frame; just flipping the values because camera is "sideways"
            ''' Yellow Marker'''
            # marker_pose_body_ref = np.array([marker_pose_cam_ref[1], -marker_pose_cam_ref[0]])

            '''Aruco Marker'''
            marker_pose_body_ref = aruco_ref_to_body_ref(marker_pose_aruco_ref, marker_tracker)
            print(marker_pose_body_ref)

            command_right = pid(marker_pose_body_ref[0])
            command_forward = pid(marker_pose_body_ref[1])

            # Send position command to the vehicle
            dronekit_utils.goto_position_target_body_offset_ned(vehicle,
                                                                forward=command_forward,
                                                                right=command_right,
                                                                down=0)

            if DEBUG:
                print("Sending: {}, {}".format(command_right, command_forward))
                pose_file.write("{} {}\n".format(marker_pose_body_ref[0], marker_pose_body_ref[1]))
                pid_file.write("{} {}\n".format(command_forward, command_right))
        else:
            if DEBUG:
                pose_file.write("{} {}\n".format("N/A", "N/A"))
                pid_file.write("{} {}\n".format("N/A", "N/A"))

def aruco_ref_to_body_ref(aruco_pose, marker_tracker):
    # Flip signs because aruco has bottom right and down as positive axis
    aruco_pose = -aruco_pose

    # Remove z-component
    aruco_pose = np.delete(aruco_pose, 2, 0)

    # Convert camera resolution from pixels to meters; reshape from dimensions (1,2) to (2,)
    cam_dimensions = np.squeeze(np.array([marker_tracker.get_resolution()]) * marker_tracker.get_scale_factor())

    return cam_dimensions / 2 - aruco_pose

def main():
    # Create Marker Detector; before UAV takes off because takes a while to process
    marker_tracker = ArucoTracker(src=args["video"],
                                  use_pi=args["picamera"],
                                  resolution=(640, 480),
                                  framerate=15,
                                  debug=args["debug"])

    # Connect to the Pixhawk
    vehicle = connect_vehicle()

    # Arm the UAV
    dronekit_utils.arm(vehicle)

    # Takeoff and fly to a target altitude
    dronekit_utils.takeoff(vehicle, TARGET_ALTITUDE)

    # Maintain hover over a marker
    marker_hover(vehicle, marker_tracker)

    # Land the UAV (imprecisely)
    dronekit_utils.land(vehicle)


if __name__ == "__main__":
    main()
