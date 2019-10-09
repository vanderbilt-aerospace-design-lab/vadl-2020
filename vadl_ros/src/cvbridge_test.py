#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ImageSub:
    def __init__(self):
        self.image_sub = rospy.Subscriber("image_raw", Image, self.callback)
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

if __name__ == "__main__":
    rospy.init_node("image_sub")
    isub = ImageSub()
    rospy.spin()
