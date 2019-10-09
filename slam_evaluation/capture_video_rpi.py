from __future__ import print_function
import cv2
import numpy as np
import argparse
from dronekit import connect, VehicleMode
import time

''' Current State of script 

VideoWriter was changed to work for whatever version of OpenCV is on the RPi. Code that
waits for the vehicle to be armed and for the GPS to localize the UAV are commented out for testing.
They should be uncommented before the real flight'''

GPS_NPY_FILE_BASE = "./gps_data/gps_pose"
CAMERA_DATA_DIR = './camera_data'
CAMERA_VIDEO_FILE = CAMERA_DATA_DIR + '/flight_0.avi'

# Connect to the Vehicle.
# Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
print("\nConnecting to vehicle:")
vehicle = connect("/dev/ttyAMA0", wait_ready=True, baud=57600)
print("Connected")

# Wait for the vehicle to be armed
#while not vehicle.armed:
 #   print("Waiting for armed")
  #  time.sleep(1)

# Wait for GPS to set home location
#while not vehicle.home_location:
 #   cmds = vehicle.commands
  #  cmds.download()
   # cmds.wait_ready()
    #print("Waiting for home location")
    #time.sleep(0.5)

ct = 0
gps_ct = 0
pose = []
start_time = time.time()

# Define video capture and writer
cap = cv2.VideoCapture(0)
fourcc = cv2.cv.CV_FOURCC(*'XVID')
out = cv2.VideoWriter(CAMERA_VIDEO_FILE, fourcc, 15.0, (640, 480))

while True:
    # Capture image
    ret, img = cap.read()

    if ret:
        
        # Write the image
        out.write(img)
        

        # Retrieve global GPS coordinates w/ relative alt from vehicle
        # Subtract the home location to obtain full relative coordinates
        location_gps = vehicle.location.global_relative_frame
        #pose_x = location_gps.lat - vehicle.home_location.lat
        #pose_y = location_gps.lon - vehicle.home_location.lon
        #pose_z = location_gps.alt

        # Debugging
        pose_x = 0
        pose_y = 0
        pose_z = 0
        pose.append([pose_x, pose_y, pose_z])

        # Save GPS pose every minute
        if int(time.time() - start_time) > 2:
            np.save(GPS_NPY_FILE_BASE + "_{}".format(gps_ct), np.array(pose))
            pose = []
            start_time = time.time()
            gps_ct += 1

        # For debugging
        if time.time() - start_time > 10:
            break
       # if not vehicle.armed:
        #    np.save(GPS_NPY_FILE_BASE + "_{}".format(gps_ct), np.array(pose))
         #   break

# When everything done, release the capture
cap.release()
out.release()
cv2.destroyAllWindows()
