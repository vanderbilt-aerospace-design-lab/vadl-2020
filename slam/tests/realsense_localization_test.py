### --------------------------------------------------------- ###
#
# This script sends Realsense position estimates to the Pixhawk. 
# Run this script to manually fly the UAV with SLAM.
#
### --------------------------------------------------------- ###


from utils import dronekit_utils
import slam_evaluation.realsense_localization as realsense_localization
import time

def main():

    # Connect to vehicle
    vehicle = dronekit_utils.connect_vehicle()

    # Start SLAM
    realsense_localization.start(vehicle)

    # Just keeping the script alive; pose estimates are sent in a 
    # background thread.
    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()
