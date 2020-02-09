from dronekit import connect, VehicleMode
import time
#import dronekit_sitl
import argparse
from utils import dronekit_utils
from slam_evaluation import realsense_localization
from apscheduler.schedulers.background import BackgroundScheduler

''' Test script for auto takeoff and landing'''

TARGET_ALTITUDE = 1 # Meters

def main():
    # Connect to the Pixhawk
    vehicle = dronekit_utils.connect_vehicle()

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
