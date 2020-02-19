
# --------------------------------------------------------------------------------- #
#
# This script is for testing distance estimation using arducam based on the
# analysis done in arducam_fov_analysis.py. Assuming the camera is at a 90 degree
# angle to a flat surface, the distance along the ground to any pixel in the image
# can be easily calcutated. This script displays an image and outputs the estimated
# distance to any pixel in the image after a mouse click on the image
#
# --------------------------------------------------------------------------------- #

from math import sqrt, tan, radians, degrees
import cv2
from matplotlib import pyplot as plt

ARDUCAM_SLOPE = 0.3222               # Pixel vs. angle (degrees) slope for Arducam with 480x640 resolution
HEIGHT = 37.4              # Height of camera off the ground
IMG_PATH = "sz.png"  # Image to display

# Callback for mouse click on image
def onclick(event):
    print(event.x, event.y)

    # Get distance from pixe to center of the image
    dist = sqrt((240 - event.y)**2 + (320 - event.x)**2)
    # print(dist)

    # Compute distance
    # print(ARDUCAM_SLOPE  * dist)

# Display image and respond to clicks
fig = plt.figure()
ax = fig.add_subplot(111)
ax.imshow(cv2.imread(IMG_PATH))
fig.canvas.mpl_connect('button_press_event', onclick)
plt.show()


