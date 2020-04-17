### ---------------------------------------------------------- ###
#
# Plot acceleration vs. time in the x, y, and z axes. Used to 
# plot vibration data received from the Realsense.
#
### ---------------------------------------------------------- ###

import matplotlib.pyplot as plt
import argparse

#Input accel file name with -f in command line. 
# The file should follow the tum format.
#Acceleration is initialized to 0 in all axes (gravity is zeroed out) 
parser = argparse.ArgumentParser(description='Plot acceleration vs. time')
parser.add_argument('-f', "--file", type=str,
                    help="Accel file name.")
args = vars(parser.parse_args())

# Open the accel file
f = open(args["file"] + ".txt", "r")
lines = f.readlines() # Read the data

time = []
accel_x = []
accel_y = []
accel_z = []

# Parse the data
for line in lines:
    line = line.split()
    time.append(float(line[0]))
    accel_x.append(float(line[1]))
    accel_y.append(float(line[2]))
    accel_z.append(float(line[3]))

# Plot the data
plt.figure()
plt.subplot(131)
plt.plot(time, accel_x)
plt.ylabel("x_accel")
plt.xlabel("time")
plt.title("X Acceleration")

plt.subplot(132)
plt.plot(time, accel_y)
plt.ylabel("y_accel")
plt.xlabel("time")
plt.title("Y Acceleration")

plt.subplot(133)
plt.plot(time, accel_z)
plt.ylabel("z_accel")
plt.xlabel("time")
plt.title("Z Acceleration")

# Save the figure
plt.savefig(args["file"] + ".jpeg")
plt.show()

