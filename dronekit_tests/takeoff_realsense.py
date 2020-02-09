from dronekit import connect, VehicleMode
import time
import dronekit_sitl
import argparse
from utils import dronekit_utils
from slam_evaluation import realsense_localization
from apscheduler.schedulers.background import BackgroundScheduler

''' Test script for auto takeoff and landing'''

TARGET_ALTITUDE = 1 # Meters

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--sitl',
                   help="Vehicle connection target string. If specified, SITL will be used.")
args = parser.parse_args()

sitl = None

#Start SITL if connection string specified
if args.sitl:
    sitl = dronekit_sitl.start_default()
    CONNECTION_STRING = sitl.connection_string()
else:
    CONNECTION_STRING = "/dev/ttyAMA0"

# Connect to the Vehicle
print("\nConnecting to vehicle on: %s" % CONNECTION_STRING)
vehicle = connect(CONNECTION_STRING, wait_ready=True, baud=921600)

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt

        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print "Reached target altitude"
            break
        time.sleep(1)


def land():
    print "Landing..."
    vehicle.mode = VehicleMode("LAND")

    # Close vehicle object before exiting script
    print("Closing vehicle object")
    vehicle.close()


def main():
    # Connect to the Pixhawk
    vehicle = dronekit_utils.connect_vehicle_args(args)

    # Create a scheduler to send Mavlink commands in the background
    sched = BackgroundScheduler()

    # Begin realsense localization in the background
    realsense_localization.start(vehicle, sched)

    # Wait for home location to be set
    while vehicle.home_location is None:
        print("Waiting for EKF Origin to be set")
        time.sleep(1)

    # Arm the UAV
    dronekit_utils.arm_realsense_mode(vehicle)

    dronekit_utils.takeoff(vehicle, TARGET_ALTITUDE)

    time.sleep(10)

    dronekit_utils.land(vehicle)

    time.sleep(100)
    time.sleep(1)


if __name__ == "__main__":
    main()
