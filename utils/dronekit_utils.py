from __future__ import print_function
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import time
import numpy as np

CONNECTION_STRING = "/dev/ttyAMA0"

def connect_vehicle(connection_string=CONNECTION_STRING):
    # Connect to the Vehicle
    print("\nConnecting to vehicle on: %s" % connection_string)

    if connection_string is None:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()
        return connect(connection_string, wait_ready=True)
    else:
        return connect(connection_string, wait_ready=True, baud=921600)

def connect_vehicle_args(args=None):
    # Start SITL if connection string specified
    if args["sitl"]:
        import dronekit_sitl
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()
    else:
        connection_string = CONNECTION_STRING

    # Connect to the Vehicle
    return connect_vehicle(connection_string)

def arm(vehicle):
    print("Basic pre-arm checks")

    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    # Do not arm until EKF has found its home location
    wait_for_home_location(vehicle)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

def arm_realsense_mode(vehicle):
    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    time.sleep(5) # So the UAV does not takeoff immediately after

def arm_no_failsafe(vehicle):
    print("Arming motors")
    vehicle.armed = True

def disarm(vehicle):
    print("Disarming vehicle")
    vehicle.armed = False

def reboot(vehicle):
    print("Rebooting FCU")
    vehicle.reboot()

def reboot_and_connect(vehicle, args=None):
    # Reboot
    reboot(vehicle)
    time.sleep(1)
    vehicle.close()

    # Wait for reboot to finish
    time.sleep(7)

    # Reconnect
    return connect_vehicle_args(args)


def takeoff(vehicle, aTargetAltitude):
    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)

        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def land(vehicle):
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")

    # Close vehicle object before exiting script
    print("Closing vehicle object")
    vehicle.close()

def rtl(vehicle):
    print("Returning to Launch")

    vehicle.mode = VehicleMode("RTL")

    # Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()

def wait_for_home_location(vehicle):
    # Wait for GPS to set home location
    while not vehicle.home_location:
        cmds = vehicle.commands
        cmds.download()
        cmds.wait_ready()
        print("Waiting for home location")
        time.sleep(0.5)

def goto_position_target_local_offset_ned(vehicle, north, east, down):
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

# TODO: If MAV_FRAME_BODY_OFFSET_NED does not work, try MAV_FRAME_BODY_FRD, since apparently the former is deprecated.
def goto_position_target_body_offset_ned(vehicle, forward, right, down):
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
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b0000111111111000,  # type_mask (only positions enabled)
        forward, right, down,  # x, y, z positions (or Foward, Right, Down in the MAV_FRAME_BODY_OFFSET_NED frame
        0, 0, 0,  # x, y, z velocity in m/s (not used)
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)

def send_body_velocity(vehicle, velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

# Send a mavlink SET_GPS_GLOBAL_ORIGIN message (http://mavlink.org/messages/common#SET_GPS_GLOBAL_ORIGIN), which allows us to use local position information without a GPS.
def set_default_global_origin(vehicle, home_lat, home_lon, home_alt):
    msg = vehicle.message_factory.set_gps_global_origin_encode(
        int(vehicle._master.source_system),
        home_lat,
        home_lon,
        home_alt
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()

# Send a mavlink SET_HOME_POSITION message (http://mavlink.org/messages/common#SET_HOME_POSITION), which allows us to use local position information without a GPS.
def set_default_home_position(vehicle, home_lat, home_lon, home_alt):
    x = 0
    y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    msg = vehicle.message_factory.set_home_position_encode(
        int(vehicle._master.source_system),
        home_lat,
        home_lon,
        home_alt,
        x,
        y,
        z,
        q,
        approach_x,
        approach_y,
        approach_z
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()

