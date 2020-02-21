''' Sample zone approach and hover

UAV will takeoff to a set altitude, search for the sample zone, navigate to it, and descend to the desired
 sampling altitude'''

import time
import argparse
from apscheduler.schedulers.background import BackgroundScheduler

# Custom packages
from utils import dronekit_utils
from slam import realsense_localization
from marker_detection.camera import Realsense

TAKEOFF_ALTITUDE = 0.5 # Meters
NAV_X = 1
NAV_Y = 0
NAV_Z = 1
AIRSPEED = 0.25 # M/s

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Fly a UAV to a set altitude and hover over a marker.')
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

args["x"] *= 0.3048
args["z"] *= 0.3048

def main():

    # Connect to the Pixhawk
    vehicle = dronekit_utils.connect_vehicle()

    rs = Realsense()

    # Create a scheduler to send Mavlink commands in the background
    sched = BackgroundScheduler()

    # Begin realsense localization in the background
    realsense_localization.start(vehicle, rs=rs, scheduler=sched)

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

    # Linear travel
    dronekit_utils.goto_position_target_body_offset_ned(vehicle,
                                                        forward=args["x"],
                                                        right=0,
                                                        down=0)
    
    time.sleep(150)
    # Return to Launch
    if args["rtl"]:
        dronekit_utils.rtl(vehicle)
    else:
        dronekit_utils.land(vehicle)

if __name__ == "__main__":
    main()
