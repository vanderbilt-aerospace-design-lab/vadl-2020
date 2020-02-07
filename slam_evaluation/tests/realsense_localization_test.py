from utils import dronekit_utils
import slam_evaluation.realsense_localization as realsense_localization
import time

def main():

    # Connect to vehicle
    vehicle = dronekit_utils.connect_vehicle(connection_string=None)
    realsense_localization.start(vehicle)

    while True:
        print("I am running in the foreground")
        time.sleep(1)

if __name__ == "__main__":
    main()