from __future__ import print_function
import time 
import argparse
from utils import dronekit_utils

### ------------------------------------------------------------------------------- ###
#
# This script will takeoff the UAV, fly to a specified location, and then return to origin and land.
# This uses the UAV body reference frame, which will usually be the desired reference frame for autonomous navigation.
# Refer to "../utils/dronekit_utils.py" for more documentation.
#
### ------------------------------------------------------------------------------- ###

TARGET_ALTITUDE = 2 # Meters; Takeoff to this altitude
AIRSPEED = 1 # UAV flight speed; m/s

# Set the desired position to move to relative to the current UAV position.
# Ex: POSITION_FORWARD = 3, POSITION_RIGHT = 5, POSITION_DOWN = 0 will move the UAV 
# 3 meters forward and 5 meters right (diagonally in real life).
POSITION_FORWARD = 3
POSITION_RIGHT = 0
POSITION_DOWN = 0

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Record GPS and Realsense data.')
parser.add_argument('--sitl',
                   help="Vehicle connection target string. If specified, SITL will be used.")

args = vars(parser.parse_args())

# Navigate the UAV to a specified location
def simple_goto(vehicle):

    print("Set default/target airspeed to {} m/s".format(AIRSPEED))
    vehicle.airspeed = AIRSPEED

    # Move the UAV
    dronekit_utils.goto_position_target_body_offset_ned(vehicle, 
                                                        forward=POSITION_FORWARD, 
                                                        right=POSITION_RIGHT, 
                                                        down=POSITION_FORWARD)


def main():

    # Connect to the Pixhawk
    vehicle = dronekit_utils.connect_vehicle_args(args)

    # Wait for GPS to initialize home location
    dronekit_utils.wait_for_home_location(vehicle)

    # Arm the vehicle
    dronekit_utils.arm(vehicle)

    # Takeoff
    dronekit_utils.takeoff(vehicle, TARGET_ALTITUDE)

    # Move the UAV
    simple_goto(vehicle)

    # Wait for the navigation command to complete
    time.sleep(10)

    # Return to Launch
    dronekit_utils.rtl(vehicle)

if __name__ == "__main__":
    main()
