from __future__ import print_function
import cv2
import numpy as np
import argparse
from dronekit import connect, VehicleMode
import time

GPS_NPY_FILE_BASE = "./gps_data/gps_pose"
CAMERA_DATA_DIR = './camera_data'
CAMERA_VIDEO_FILE = CAMERA_DATA_DIR + '/flight_0.avi'

# Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle.
# Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
print("\nConnecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True)

# Wait for the vehicle to be armed
while not vehicle.armed:
    print("Waiting for armed")
    time.sleep(1)

# Wait for GPS to set home location
while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    print("Waiting for home location")
    time.sleep(0.5)

ct = 0
gps_ct = 0
pose = []
start_time = time.time()

# Define video capture and writer
cap = cv2.VideoCapture(1)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(CAMERA_VIDEO_FILE, fourcc, 30.0, (640, 480))

while True:
    # Capture image
    ret, img = cap.read()

    if ret:

        # Write the image
        out.write(img)

        # cv2.imshow('frame', img)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break

        # Retrieve global GPS coordinates w/ relative alt from vehicle
        # Subtract the home location to obtain full relative coordinates
        location_gps = vehicle.location.global_relative_frame
        pose_x = location_gps.lat - vehicle.home_location.lat
        pose_y = location_gps.lon - vehicle.home_location.lon
        pose_z = location_gps.alt
        pose.append([pose_x, pose_y, pose_z])

        # Save GPS pose every minute
        if int(time.time() - start_time) > 2:
            np.save(GPS_NPY_FILE_BASE + "_{}".format(gps_ct), np.array(pose))
            pose = []
            start_time = time.time()
            gps_ct += 1

        if not vehicle.armed:
            break

# When everything done, release the capture
cap.release()
out.release()
cv2.destroyAllWindows()
