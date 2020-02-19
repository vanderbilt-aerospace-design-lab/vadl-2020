
from utils.dronekit_utils import *
from utils.controller_utils import *
from marker_detection import camera as cam
import time
import os
import cv2

# PARAMS
SEARCH_ALTITUDE = 10 * 	0.3048
LOG_DIR = "/home/search_logs"

# Takeoff and ascend to search altitude
if __name__ == "__main__":

    # Connect to flight controller
    vehicle = connect_vehicle()

    # Connect to RC controller
    rc_controller = Controller(vehicle)

    # Start a video stream
    vc = cam.VideoStreamer(use_pi=1, framerate=10)

    # Arm and take off to search altitude
    arm(vehicle)
    takeoff(vehicle, 2)

    # Ascend to search altitude
    goto_position_target_local_ned(vehicle, 0, 0, -SEARCH_ALTITUDE)
    time.sleep(5)

    # Store data from search
    timestamp = str(int(time.time()))
    new_log_dir = LOG_DIR + "/log_" + timestamp
    os.mkdir(new_log_dir)
    datalog = open(new_log_dir + "/" + "attitude_data" + ".txt")

    # Search routine
    for heading in [0, 45, 90, 135, 180, 225, 270, 315]:

        # Yaw vehicle to next heading
        condition_yaw(vehicle, heading)

        # Wait for it to get there
        time.sleep(2)

        # Take picture
        cv2.imwrite(new_log_dir + "/" + "heading_" + str(heading) + ".jpg", vc.read())

        # Save attitude data
        datalog.write(str(heading) + ",")
        datalog.write(str(vehicle.attitude.roll) + ",")
        datalog.write(str(vehicle.attitude.pitch) + ",")
        datalog.write(str(vehicle.attitude.yaw) + ",")
        datalog.write(str(vehicle.location.global_relative_frame.alt) + "\n")

    # Land and disarm
    land(vehicle)



