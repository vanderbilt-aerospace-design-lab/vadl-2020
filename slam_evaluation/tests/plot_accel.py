import matplotlib.pyplot as plt
import argparse

''' Input accel file name with -f in command line. File should have the format as coded in t265_to_mavlink.py. 
Acceleration is initialized to 0 in all axes (gravity is zeroed out) '''

parser = argparse.ArgumentParser(description='Reboots vehicle')
parser.add_argument('-f', "--file", type=str,
                    help="Accel file name.")

args = vars(parser.parse_args())

f = open(args["file"], "r")
lines = f.readlines()

time = []
accel_x = []
accel_y = []
accel_z = []
for line in lines:
    line = line.split()
    time.append(float(line[0]))
    accel_x.append(float(line[1]))
    accel_y.append(float(line[2]))
    accel_z.append(float(line[3]))

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
plt.show()

