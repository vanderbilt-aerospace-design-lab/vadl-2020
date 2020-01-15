from __future__ import print_function
import time 
import argparse
from utils import dronekit_utils

TARGET_ALTITUDE = 2 # Meters
AIRSPEED = 1

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Record GPS and Realsense data.')
parser.add_argument('--sitl',
                   help="Vehicle connection target string. If specified, SITL will be used.")

args = vars(parser.parse_args())

def simple_goto(vehicle):
    print("Set default/target airspeed to {} m/s".format(AIRSPEED))
    vehicle.airspeed = AIRSPEED
    print(vehicle.location.local_frame)

    # Move 3 m forward
    dronekit_utils.goto_position_target_body_offset_ned(vehicle, forward=3, right=0, down=0)
    time.sleep(10)

    print(vehicle.location.local_frame)

    # Move 3 m right
    dronekit_utils.goto_position_target_body_offset_ned(vehicle, forward=0, right=3, down=0)
    time.sleep(10)

    print(vehicle.location.local_frame)

def main():
    # Connect to the Pixhawk
    vehicle = dronekit_utils.connect_vehicle_args(args)

    # Wait for GPS to initialize home location
    dronekit_utils.wait_for_home_location(vehicle)

    # Arm the vehicle
    dronekit_utils.arm(vehicle)

    # Takeoff
    dronekit_utils.takeoff(vehicle, TARGET_ALTITUDE)

    simple_goto(vehicle)

    # Return to Launch
    dronekit_utils.rtl(vehicle)

if __name__ == "__main__":
    main()
