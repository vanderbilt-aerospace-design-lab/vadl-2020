from dronekit import connect, VehicleMode
import time
import dronekit_sitl
import argparse

### Test script for auto takeoff and landing ###

TARGET_ALTITUDE = 1 # Meters; Takeoff to this altitude

#Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Test guided takeoff')
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

# Arms the vehicle and flys to a target altitude
def arm_and_takeoff(target_altitude):
    
    print("Basic pre-arm checks")

    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")

    # Copter should ALWAYS arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True # Arm the vehicle

    # Confirm the vehicle is armed before attempting to take off
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print("Altitude: ", vehicle.location.global_relative_frame.alt)

        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


# Land the UAV directly below where it is hovering
def land():
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")

    # Close vehicle object before exiting script
    print("Closing vehicle object")
    vehicle.close()


def main():

    # Arm the vehicle and takeoff to the desired altitude
    arm_and_takeoff(TARGET_ALTITUDE)

    # Sleep; Dronekit will not wait for commands to finish before sending another.
    # You must pause the script long enough to finish commands
    time.sleep(15)

    # Land directly below where the UAV is hovering
    land()


if __name__ == "__main__":
    main()
