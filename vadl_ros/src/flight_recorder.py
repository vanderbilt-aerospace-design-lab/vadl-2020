#!/usr/bin/env python
from __future__ import print_function

# ROS
import rospy
import roslib
# roslib.load_manifest('my_package')
import std_msgs.msg
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError

# Other packages
import cv2
import numpy as np
from dronekit import connect
import dronekit_sitl
import time
import argparse

# Global parameters
CAMERA_DATA_DIR = './flight_videos'
CAMERA_VIDEO_FILE = CAMERA_DATA_DIR + '/flight_0.avi'

# Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
parser.add_argument('--connect',
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

# connection_string = None
# sitl = None


class FlightRecorder:

    def __init__(self):
        self.connection_string = None

        # Publishers
        self.image_pub = rospy.Publisher("image_raw", Image, queue_size=0)
        self.gps_pub = rospy.Publisher("gps_rel_pose", PointStamped, queue_size=0)
        self.rate = rospy.Rate(30) # Hz

        # OpenCV bridge
        self.bridge = CvBridge()

        # Establish connection to Pixhawk
        # Start SITL if no connection string specified
        if not self.connection_string:
            self.sitl = dronekit_sitl.start_default()
            self.connection_string = self.sitl.connection_string()

        # Connect to the Vehicle.
        # Set `wait_ready=True` to ensure default attributes are populated before `connect()` returns.
        print("\nConnecting to vehicle on: %s" % self.connection_string)
        self.vehicle = connect(self.connection_string, wait_ready=True)

        # Wait for the vehicle to be armed
        # while not self.vehicle.armed:
        #     print("Waiting for armed")
        #     time.sleep(1)

        # Wait for GPS to set home location
        while not self.vehicle.home_location:
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()
            print("Waiting for home location")
            time.sleep(0.5)

        # Set up OpenCV video capture
        self.cap = cv2.VideoCapture(0)
        # print("Codec: {}".format(self.cap.get(6)))
        # self.cap.set(6, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.out = cv2.VideoWriter("flight.avi", self.fourcc, 15.0, (640, 480))
        # print("OpenCV video established")
        # print("Frame rate: {}".format(self.cap.get(5)))
        # print("Pixels: {}, {}".format(self.cap.get(3), self.cap.get(4)))
        # print("Codec: {}".format(self.cap.get(6)))

        # Start taking data
        self.record()


    def record(self):
        print("Recording data...")
        # while self.vehicle.armed:
        while not rospy.is_shutdown():

            # Capture image
            ret, cv_image = self.cap.read()

            if ret:

                # Write the image to video
                self.out.write(cv_image)

                # Convert cv2 image to ROS image
                ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")

                # Retrieve global GPS coordinates w/ relative altitude from vehicle
                # Subtract the home location to obtain full relative coordinates
                gps_pose = PointStamped()

                # Header
                gps_pose.header.stamp = rospy.Time.now()

                # Data
                location_gps = self.vehicle.location.global_relative_frame
                gps_pose.point.x = location_gps.lat - self.vehicle.home_location.lat
                gps_pose.point.y = location_gps.lon - self.vehicle.home_location.lon
                gps_pose.point.z = location_gps.alt

                # Publish image
                self.image_pub.publish(ros_image)

                # Publish GPS pose
                self.gps_pub.publish(gps_pose)

        # Shut down openCV gracefully after vehicle is disarmed
        self.cap.release()
        self.out.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("flight_recorder")
    flight_recorder = FlightRecorder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

        # Shut down openCV if program is cancelled
        flight_recorder.cap.release()
        flight_recorder.out.release()
        cv2.destroyAllWindows()
