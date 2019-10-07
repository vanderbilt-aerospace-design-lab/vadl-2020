import numpy as np  

SLAM_ODOM_FILE = "/home/vadl/catkin_ws/src/VINS-Mono/vins_estimator/odom_data.txt"
GPS_ODOM_FILE = "/home/vadl/catkin_ws/src/vadl-2020/slam_evaluation/gps_data/gps_odom.txt"

# Load data and format into arrays
# TODO: Finish this part after data is collected and format is known
with open(SLAM_ODOM_FILE, "r") as f_slam, open(GPS_ODOM_FILE) as f_gps:
    
    slam_odom = f_slam.readlines()
    gps_odom = f_gps.readlines()

    for x,y in slam_odom, gps_odom:
        print(x)
        print(y)

    f_slam.close()
    f_gps.close()

# At this point, data is formatted and synced up based on frequency and when data was taken
slam_odom = np.array([])
gps_odom = np.array([])

# Calculate the sum of the squared difference
sum_of_difference_squared = 0
for i in range(0, slam_odom.shape[0]):
    sum_of_difference_squared += np.power((slam_odom[i] - gps_odom[i]), 2)

# Mean
mean = sum_of_difference_squared / slam_odom.shape[0]

# RMSE
rmse = np.sqrt(mean)

print("RMSE: {}".format(rmse))


