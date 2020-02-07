from utils import dronekit_utils
import slam_evaluation.realsense_localization as realsense_localization
import time

def main():

    # Connect to vehicle
    vehicle = dronekit_utils.connect_vehicle()
    realsense_localization.start(vehicle)

    while True:
        time.sleep(1)

if __name__ == "__main__":
    main()
