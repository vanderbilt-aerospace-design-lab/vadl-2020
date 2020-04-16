import time
from utils import dronekit_utils
from slam import realsense_localization
from apscheduler.schedulers.background import BackgroundScheduler

### -------------------------------------------------------------------------------- ###
#
# This script will takeoff the UAV, move it to a specified position, and return to launch and land while 
# localizing with the Realsense. This is nearly the same as the "simple_goto_body" script, but it establishes
# Realsense message communication with the Pixhawk and arms without safety checks.
#
### -------------------------------------------------------------------------------- ###


TARGET_ALTITUDE = 1 # Meters; Takeoff to this altitude
AIRSPEED = 0.25 # M/s; UAV flight speed

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

    # Takeoff to a desired altitude
    dronekit_utils.takeoff(vehicle, TARGET_ALTITUDE)

    # Wait for takeoff to complete
    time.sleep(6)

    # Set the UAV airspeed
    vehicle.airspeed = AIRSPEED

    # Move the UAV in the body NED offset reference frame
    # Navigation commands are relative to the current UAV position 
    # and Forward, Right, Down are relative to the Pixhawk orientation.
    dronekit_utils.goto_position_target_body_offset_ned(vehicle,
                                                       forward=0.5,
                                                       right=0,
                                                       down=0)

    # Wait for navigation command to finish
    time.sleep(7)

    # Land the vehicle
    dronekit_utils.rtl(vehicle)

    time.sleep(100)


if __name__ == "__main__":
    main()
