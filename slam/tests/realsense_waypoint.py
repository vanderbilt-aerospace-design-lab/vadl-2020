### --------------------------------------------------------------------- ###
# 
# This script commands the UAV to takeoff, fly to a defined distance, and 
# return to its origin and land. This is done in SLAM mode.
# Call the script with the correct args to set the flight path.
# 
# Example: 
#
# sudo python3 slam/tests/realsense_waypoint.py -x 120 -y 10 -z 2
# 
# This will first fly the UAV 2 meters UP, then 120 meters forward, 
# and then 10 meters right
### --------------------------------------------------------------------- ###


import time
import argparse
from apscheduler.schedulers.background import BackgroundScheduler

# Custom packages
from utils import dronekit_utils
from slam import realsense_localization
from marker_detection.camera import Realsense

# Follows body reference frame (Forward, Right, Down)
TAKEOFF_ALTITUDE = 0.5 # Meters; Takeoff to this altitude
NAV_X = 1 # Meters; Default fly forward (+ve)
NAV_Y = 0 # Meters; Default fly to the right (+ve)
NAV_Z = 1 # Meters; Default fly down (+ve)
AIRSPEED = 0.25 # M/s

#Set up arg parsing
parser = argparse.ArgumentParser(description='Fly a UAV in SLAM mode a set distance and RTL.')
parser.add_argument('-x', type=int, default=NAV_X,
                    help="Linear distance to fly.")
parser.add_argument('-y', type=int, default=NAV_Y,
                    help="Linear distance to fly.")
parser.add_argument('-z', type=int, default=NAV_Z,
                    help="Linear distance to fly.")
parser.add_argument('-s', '--speed', type=int, default=AIRSPEED,
                    help="Linear distance to fly.")
parser.add_argument('--rtl', type=int, default=0,
                    help="RTL will be used. Land otherwise.")

args = vars(parser.parse_args())


def main():

    # Connect to the Pixhawk
    vehicle = dronekit_utils.connect_vehicle()

    # Create a Realsense object
    rs = Realsense()

    # Create a scheduler to send Mavlink commands in the background
    sched = BackgroundScheduler()

    # Start SLAM
    realsense_localization.start(vehicle, rs=rs, scheduler=sched)

    # Wait for SLAM to initialize
    time.sleep(10)

    # Set the UAV speed
    vehicle.airspeed = args["speed"]

    # Arm the UAV
    dronekit_utils.arm_realsense_mode(vehicle)

    # Takeoff and fly to a target altitude
    dronekit_utils.takeoff(vehicle, TAKEOFF_ALTITUDE)

    # Give time to stabilize in flight
    time.sleep(2)

    # Ascend
    dronekit_utils.goto_position_target_body_offset_ned(vehicle,
                                                        forward=0,
                                                        right=0,
                                                        down=-args["z"])
    
    time.sleep(20)

    # Linear travel forward
    dronekit_utils.goto_position_target_body_offset_ned(vehicle,
                                                        forward=args["x"],
                                                        right=0,
                                                        down=0)

    # Linear travel right
    dronekit_utils.goto_position_target_body_offset_ned(vehicle,
                                                        forward=0,
                                                        right=args["y"],
                                                        down=0)
    
    # Wait for UAV to move
    time.sleep(150)

    # Return to Launch or Land
    if args["rtl"]:
        dronekit_utils.rtl(vehicle)
    else:
        dronekit_utils.land(vehicle)

if __name__ == "__main__":
    main()
