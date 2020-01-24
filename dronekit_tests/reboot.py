import time
import argparse
from utils import dronekit_utils

''' Test script for auto reboot and reconnection'''

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--sitl',
                   help="Vehicle connection target string. If specified, SITL will be used.")
args = vars(parser.parse_args())

def main():

    # Connect to vehicle
    vehicle = dronekit_utils.connect_vehicle_args(args)

    # Reboot
    dronekit_utils.reboot(vehicle)
    time.sleep(1)
    vehicle.close()

    time.sleep(7)

    # Reconnect
    vehicle = dronekit_utils.connect_vehicle_args(args)
    print("Success: {}".format(vehicle.version)) 

if __name__ == "__main__":
    main()
