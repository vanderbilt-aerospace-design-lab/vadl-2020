import time
from utils import dronekit_utils
from slam import realsense_localization
from apscheduler.schedulers.background import BackgroundScheduler

### --------------------------------------------------------------------------------------------------------------------- ###
# 
# Test script for auto takeoff and landing while flying with the Realsense.
# To fly localizing off the Realsense instead of the GPS, the onboard computer must send
# position estimate messages to the Pixhawk in the background. Refer to "../slam/realsense_localization.py" for 
# more details. The Pixhawk is not able to pass all pre-arm checks with the Realsense since it (wrongly) believes there
# to be a compass error. This means that the Pixhawk must be armed slightly differently.
### ---------------------------------------------------------------------------------------------------------------------- ###

TARGET_ALTITUDE = 1 # Meters

def main():

    # Connect to the Pixhawk
    vehicle = dronekit_utils.connect_vehicle()

    # Create a scheduler to send Mavlink commands in the background
    sched = BackgroundScheduler()

    # Begin realsense localization; send vision position estimates to the Pixhawk
    realsense_localization.start(vehicle, sched)

    # Wait for home location to be set
    # If the home location is not set before attempting to arm, the Pixhawk will 
    # not allow arming. This is still very finicky - sometimes the home location will
    # refuse to be set. If this happens, restart the UAV and try again. (There is a probably a 
    # more elegant solution to this, but it has not been found.)
    while vehicle.home_location is None:
        print("Waiting for EKF Origin to be set")
        time.sleep(1)

    # Arm the UAV; NO SAFETY CHECKS
    dronekit_utils.arm_realsense_mode(vehicle)

    # Takeoff to a target altitude
    dronekit_utils.takeoff(vehicle, TARGET_ALTITUDE)

    # Sleep so the takeoff command finishes
    time.sleep(10)

    # Land the UAV
    dronekit_utils.land(vehicle)

    time.sleep(100)
    time.sleep(1)


if __name__ == "__main__":
    main()
