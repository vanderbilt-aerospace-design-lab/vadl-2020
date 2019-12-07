#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from __future__ import print_function
import time
import dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationLocal

# Connect to the Vehicle
TARGET_ALTITUDE = 3 # Meters
CONNECTION_STRING = "/dev/ttyAMA0"
print('Connecting to vehicle on: %s' % CONNECTION_STRING)
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
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def simple_goto():
    print("Set default/target airspeed to 3")
    vehicle.airspeed = 3

    print("Going towards first point for 30 seconds ...")
    lat = vehicle.location.global_relative_frame.lat + 2
    lon = vehicle.location.global_relative_frame.lon + 2
    point1 = LocationGlobalRelative(lat, lon, vehicle.location.global_relative_frame.alt)
    # point1 = LocationLocal(2, 2, 0)
    vehicle.simple_goto(point1)

    # sleep so we can see the change in map
    time.sleep(30)

    print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
    point2 = LocationGlobalRelative(-35.363244, 149.168801, 20)
    vehicle.simple_goto(point2, groundspeed=10)

    # sleep so we can see the change in map
    time.sleep(30)


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
