from dronekit import connect, VehicleMode
import time
import argparse

### --------------------------------------------------------------------- ###
# This script allows you to test that manual override with a remote controller works.
# It is ESSENTIAL to always have manual override available, since the Pixhawk will sometimes
# zoom the UAV into space without your consent. Be alert. Be ready. 
#
### --------------------------------------------------------------------- ###

TARGET_ALTITUDE = 2 # Meters; Takeoff to this altitude

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Record GPS and Realsense data.')
parser.add_argument('--sitl',
                   help="Vehicle connection target string. If specified, SITL will be used.")

args = vars(parser.parse_args())

CONNECTION_STRING = ""
# Connect to the Vehicle (in this case a simulator running the same computer)
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)


def main():

    # Connect to the Pixhawk
    vehicle = dronekit_utils.connect_vehicle_args(args)

    # Wait for GPS to initialize home location
    dronekit_utils.wait_for_home_location(vehicle)

    # Arm the vehicle
    dronekit_utils.arm(vehicle)

    # Takeoff to a desired altitude
    dronekit_utils.takeoff(vehicle, TARGET_ALTITUDE)

    # Manual override while the UAV is hovering. The Pixhawk is always waiting for 
    # manual override from a remote controller, so just switch out of GUIDED mode to 
    # LOITER, ALT HOLD, or STABILIZE to manually fly the UAV.
    time.sleep(2000)


if __name__ == "__main__":
    main()
