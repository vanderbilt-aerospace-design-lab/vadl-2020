''' Flight landing script
    Goal is to takeoff, fly to a set altitude,
    switch to tracking mode, and maintain a
    hover over the marker. Manual override always available. '''

from dronekit import VehicleMode
import cv2
import argparse
import time
import numpy as np
from simple_pid import PID

from utils import dronekit_utils
from marker_detection import MarkerDetector
from aruco_tracker import ArucoTracker

RPI = 0
DEBUG = 0
SILENT_DEBUG = 1

TARGET_ALTITUDE = 2 # Meters

# Files
VIDEO_FILE_SAVE = 'videos/marker_hover_1.mp4'
POSE_FILE = "pose_data/marker_pose_0.txt"
PID_FILE = "pid_data/pid_0.txt"

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--sitl',
                   help="Vehicle connection target string. If specified, SITL will be used.")
parser.add_argument('-v',
                   help="Play video instead of live stream.")
args = parser.parse_args()

if args.v is not None:
    VIDEO_FILE_STREAM = args.v
else:
    VIDEO_FILE_STREAM = 0

def connect_vehicle():
    #Start SITL if connection string specified
    if args.sitl:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        CONNECTION_STRING = sitl.connection_string()
    else:
        CONNECTION_STRING = "/dev/ttyAMA0"

    # Connect to the Vehicle
    return dronekit_utils.connect_vehicle(CONNECTION_STRING)


def marker_hover(vehicle):
    out = None
    pose_file = None
    pid_file = None

    pid = PID(1, 0.1, 0.05, setpoint=0)
    pid.sample_time = 0.01

    # Create Marker Detector
    marker_detector = ArucoTracker(DEBUG)

    # Read images from camera
    cap = cv2.VideoCapture(VIDEO_FILE_STREAM)

    if DEBUG:

        # Create video
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        out = cv2.VideoWriter(VIDEO_FILE_SAVE, fourcc, marker_detector.frame_rate, marker_detector.resolution)

        # Open text file to store UAV position
        pose_file = open(POSE_FILE, "w")
        pid_file = open(PID_FILE, "w")


    # Hover until manual override
    while vehicle.mode == VehicleMode("GUIDED"):
        ret, img = cap.read()

        # Detect marker and determine error from camera center
        marker_found = marker_detector.track_marker(img, alt=vehicle.location.global_relative_frame.alt)

        # print("Vehicle: {}, {}".format(vehicle.location.local_frame.north, vehicle.location.local_frame.east))
        if marker_found:
            marker_pose_cam_ref = marker_detector.get_marker_pose()

            # Convert to UAV body reference frame; just flipping the values because camera is "sideways"
            ''' Yellow Marker'''
            # marker_pose_body_ref = np.array([marker_pose_cam_ref[1], -marker_pose_cam_ref[0]])

            '''Aruco Marker'''
            marker_pose_body_ref = np.array([-marker_pose_cam_ref[0], -marker_pose_cam_ref[1]])

            command_right = pid(marker_pose_body_ref[0])
            command_forward = pid(marker_pose_body_ref[1])

            # print("Sending: {}, {}".format(command_right, command_forward))

            # Send position command to the vehicle
            dronekit_utils.goto_position_target_body_offset_ned(vehicle, forward=command_forward, right=command_right, down=0)

            if DEBUG:
                print(marker_detector.get_detected_image())
                out.write(marker_detector.get_detected_image())
                pose_file.write("{} {}\n".format(marker_pose_body_ref[0], marker_pose_body_ref[1]))
                pid_file.write("{} {}\n".format(command_forward, command_right))
        else:
            if DEBUG:
                print(marker_detector.get_detected_image())
                out.write(marker_detector.get_detected_image())
                pose_file.write("{} {}\n".format("N/A", "N/A"))
                pid_file.write("{} {}\n".format("N/A", "N/A"))


    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()


def marker_hover_rpi(vehicle):
    writer = None
    pose_file = None
    pid_file = None

    pid = PID(1, 0.1, 0.05, setpoint=0)
    pid.sample_time = 0.01

    # Create Marker Detector
    marker_detector = ArucoTracker(DEBUG)

    # Picamera dependencies
    from picamera.array import PiRGBArray
    from picamera import PiCamera

    # Initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = marker_detector.resolution
    raw_capture = PiRGBArray(camera, size=marker_detector.resolution)

    # Allow the camera to warm up
    time.sleep(0.1)

    if DEBUG or SILENT_DEBUG:
        # Create video
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')

        # Open text file to store UAV position
        pose_file = open("pose_data/marker_pose.txt", "w")
        pid_file = open(PID_FILE, "w")

    # Hover until manual override
    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        img = frame.array

        # Clear the stream in preparation for the next frame
        raw_capture.truncate(0)

        # Detect marker and determine error from camera center
        marker_found = marker_detector.track_marker(img, alt=vehicle.location.global_relative_frame.alt)

        # print("Vehicle: {}, {}".format(vehicle.location.local_frame.north, vehicle.location.local_frame.east))
        if marker_found:
            marker_pose_cam_ref = marker_detector.get_marker_pose()

            # Convert to UAV body reference frame; just flipping the values because camera is "sideways"
            ''' Yellow Marker'''
            # marker_pose_body_ref = np.array([marker_pose_cam_ref[1], -marker_pose_cam_ref[0]])

            '''Aruco Marker'''
            marker_pose_body_ref = np.array([-marker_pose_cam_ref[0], -marker_pose_cam_ref[1]])

            command_right = pid(marker_pose_body_ref[0])
            command_forward = pid(marker_pose_body_ref[1])

            # print("Sending: {}, {}".format(command_right, command_forward))

            # Send position command to the vehicle
            dronekit_utils.goto_position_target_body_offset_ned(vehicle, forward=command_forward, right=command_right,
                                                                down=0)

            if writer is None:
                (h, w) = frame.shape[:2]
                writer = cv2.VideoWriter(VIDEO_FILE_SAVE, fourcc, marker_detector.frame_rate, (w, h), True)

            if DEBUG or SILENT_DEBUG:
                writer.write(marker_detector.get_detected_image())
                pose_file.write("{} {}\n".format(marker_pose_body_ref[0], marker_pose_body_ref[1]))
                pid_file.write("{} {}\n".format(command_forward, command_right))

            # Option to quit the program
            if vehicle.mode is not "GUIDED":
                break
        else:
            if DEBUG or SILENT_DEBUG:
                writer.write(marker_detector.get_detected_image())
                pose_file.write("{} {}\n".format("N/A", "N/A"))
                pid_file.write("{} {}\n".format("N/A", "N/A"))

    # When everything done, release the capture
    writer.release()

def main():

    # Connect to the Pixhawk
    vehicle = connect_vehicle()

    # Arm the UAV
    dronekit_utils.arm(vehicle)

    # Takeoff and fly to a target altitude
    dronekit_utils.takeoff(vehicle, TARGET_ALTITUDE)

    # Maintain hover over a marker
    if RPI:
        marker_hover_rpi(vehicle)
    else:
        marker_hover(vehicle)

    # Land the UAV (imprecisely)
    dronekit_utils.land(vehicle)


if __name__ == "__main__":
    main()
