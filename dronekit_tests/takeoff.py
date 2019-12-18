from dronekit import connect, VehicleMode
import time
import dronekit_sitl
import argparse

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

# Vehicle callback to enable manual override
@vehicle.on_attribute('mode')
def decorated_mode_callback(self, attr_name, value):
    # `attr_name` is the observed attribute (used if callback is used for multiple attributes)
    # `attr_name` - the observed attribute (used if callback is used for multiple attributes)
    # `value` is the updated attribute value.
    print(" CALLBACK: Mode changed to", value)

    # Close vehicle object before exiting script
    print("Closing vehicle object")
    vehicle.close()
    exit(0)


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
    arm_and_takeoff(TARGET_ALTITUDE)
    time.sleep(15)
    land()


if __name__ == "__main__":
    main()
