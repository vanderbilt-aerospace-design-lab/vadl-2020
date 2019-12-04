from dronekit import connect, VehicleMode
import time

''' Test script for auto takeoff and landing'''

CONNECTION_STRING = ""
# Connect to the Vehicle (in this case a simulator running the same computer)
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

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


def arm_and_wait():

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

    while 1:
        print("Waiting for override")
        time.sleep(1)


def main():
    arm_and_wait()


if __name__ == "__main__":
    main()
