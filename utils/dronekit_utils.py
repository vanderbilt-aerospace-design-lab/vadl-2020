from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time

def connect_vehicle(connection_string):
    # Connect to the Vehicle
    print("\nConnecting to vehicle on: %s" % connection_string)

    if connection_string is None:
        return connect('tcp:127.0.0.1:5760', wait_ready=True)
    else:
        return connect(connection_string, wait_ready=True, baud=921600)


def arm(vehicle):
    print "Basic pre-arm checks"

    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True


def takeoff(vehicle, aTargetAltitude):
    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt

        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print "Reached target altitude"
            break
        time.sleep(1)


def land(vehicle):
    print "Landing..."
    vehicle.mode = VehicleMode("LAND")

    # Close vehicle object before exiting script
    print("Closing vehicle object")
    vehicle.close()

def goto_position_target_local_ned(vehicle, north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    It is important to remember that in this frame, positive altitudes are entered as negative
    "Down" values. So if down is "10", this will be 10 metres below the home altitude.
    Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
    ignored. For more information see:
    http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned
    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, # time_boot_ms (not used)
        0, 0, # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down, # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
        0, 0, 0, # x, y, z velocity in m/s (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0) # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)

