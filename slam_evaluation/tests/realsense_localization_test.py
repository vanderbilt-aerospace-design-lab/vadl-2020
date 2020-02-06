from utils import dronekit_utils
from slam_evaluation.realsense_localization import RealsenseLocalization
import time

def main():

    # Connect to vehicle
    vehicle = dronekit_utils.connect_vehicle()

    rl = RealsenseLocalization(vehicle=vehicle)

    while True:
        print("I am running")
        time.sleep(1)

if __name__ == "__main__":
    main()