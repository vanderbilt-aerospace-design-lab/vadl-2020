#!/usr/bin/env python

# -*- coding: utf-8 -*-
""" © Copyright 2015-2016, 3D Robotics. simple_goto.py: GUIDED mode "simple goto" example (Copter Only)
 Demonstrates how to arm and takeoff in Copter and how to navigate to points 
using Vehicle.simple_goto. Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html """ 


### This script uses some functions created by Dronekit. An easier way to send navigation commands is with the functions in 
# "../utils/dronekit_utils.py." Refer to simple_goto_body.py for an example of using those utils. ###

from __future__ import print_function
import time 
import math 
import argparse 
from pymavlink import mavutil 
import dronekit 
 
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationLocal, LocationGlobal

CONNECTION_STRING = "/dev/ttyAMA0"
TARGET_ALTITUDE = 2 # Takeoff to this altitude; meters

# Position the UAV will fly to. This is in the NED local coordinate system.
# North East Down but relative to the UAV current position.
NED_POSITION = (3, 0, 0)

AIRSPEED = 1 # m/s; flight speed of the UAV

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % CONNECTION_STRING)
vehicle = connect(CONNECTION_STRING, wait_ready=True, baud=921600)

#   Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
#    specified `original_location`. The returned LocationGlobal has the same `alt` value
#    as `original_location`.
#    The function is useful when you want to move the vehicle around specifying locations relative to
#    the current vehicle position.
#    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
#    For more information see:
#    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
def get_location_metres(original_location, dNorth, dEast):
    
    earth_radius = 6378137.0 # Radius of "spherical" earth

    # Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    # New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)

    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation


# Returns the ground distance in metres between two LocationGlobal objects.
# This method is an approximation, and will not be accurate over large distances and close to the
# earth's poles. It comes from the ArduPilot test code:
# https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
def get_distance_metres(aLocation1, aLocation2):
    
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
 

# Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
# location in the North, East, Down frame.
# It is important to remember that in this frame, positive altitudes are entered as negative
# "Down" values. So if down is "10", this will be 10 metres below the home altitude.
# Starting from AC3.3 the method respects the frame setting. Prior to that the frame was
# ignored. For more information see:
# http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned
# See the above link for information on the type_mask (0=enable, 1=ignore).
# At time of writing, acceleration and yaw bits are ignored.
def goto_position_target_local_ned(north, east, down):
    
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
 
    # Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.
    # The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for
    # the target position. This allows it to be called with different position-setting commands.
    # By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().
    # The method reports the distance to target every two seconds.
def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
   
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)

    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation print "DEBUG: targetLocation: %s" % targetDistance
    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
        #print "DEBUG: mode: %s" % vehicle.mode.name
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
            print("Reached target")
            break
        time.sleep(2)
        
# Arm the UAV and takeoff to a desired altitude
def arm_and_takeoff(target_altitude):

    print("Basic pre-arm checks")

    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")

    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# Navigate the UAV to the desired position.
def simple_goto():
    print("Set default/target airspeed to {} m/s".format(AIRSPEED))
    vehicle.airspeed = AIRSPEED

    goto_position_target_local_ned(NED_POSITION) #sleep so we can see the change in map
    time.sleep(15)
 
# UAV will return to its origin and land.
def rtl():
    print("Returning to Launch")

    vehicle.mode = VehicleMode("RTL")

    # Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()
 
def main():
    arm_and_takeoff(TARGET_ALTITUDE)
    simple_goto()
    rtl() 

if __name__ == "__main__":
    main()
