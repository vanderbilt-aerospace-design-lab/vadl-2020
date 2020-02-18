
import numpy
from matplotlib import pyplot as plt
import cv2
from math import atan2, degrees

# Function to extract coordinates of clicks on an image
def get_clicks(img):

    data = []

    # Click event handler
    def onclick(event, arr):
        arr += [[event.x, event.y]]
        print([event.x, event.y])

    # Retrieve clicks from image
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.imshow(img)
    fig.canvas.mpl_connect('button_press_event', lambda event: onclick(event, data))
    plt.show()

    return data

# Get coordinates of ticks from each image
long_img = cv2.imread("fov_analysis_long.jpg")
short_img = cv2.imread("fov_analysis_short.jpg")
print(type(long_img))
long_axis = get_clicks(long_img)
short_axis = get_clicks(short_img)

# Keep x or y based on axis
long_axis = [coord[0] for coord in long_axis]
short_axis = [coord[1] for coord in short_axis]

# Adjust values to start at zero
offset_long = long_axis[0]
offset_short = short_axis[0]
long_axis = [abs(i - offset_long) for i in long_axis]
short_axis = [abs(i  -offset_short) for i in short_axis]

# Height of camera off the ground
Z_HEIGHT = 2 + (13/16)

# Angle corresponding to each tick
long_axis_angles = [90 - degrees(atan2(Z_HEIGHT, i)) for i in range(len(long_axis))]
short_axis_angles = [90 - degrees(atan2(Z_HEIGHT, i)) for i in range(len(short_axis))]

m_short = (short_axis_angles[-1] - short_axis_angles[0]) / (short_axis[-1] - short_axis[0])
m_long = (long_axis_angles[-1] - long_axis_angles[0]) / (long_axis[-1] - long_axis[0])
m_avg = (m_short + m_long) / 2
fit_line = [m_avg * val for val in long_axis]

# Plot pixel vs angle
plt.subplot(121)
plt.plot(long_axis)
plt.plot(short_axis)
plt.subplot(122)
plt.plot(long_axis, long_axis_angles)
plt.plot(short_axis, short_axis_angles)
plt.plot(long_axis, fit_line)
plt.show()

# Slope of pixel vs. angle plot is what we care about!
print(m_avg)





