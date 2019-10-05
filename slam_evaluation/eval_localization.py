import numpy as np  

SLAM_ODOM_FILE = "/home/vadl/catkin_ws/src/VINS-Mono/vins_estimator/odom_data.txt"
GPS_ODOM_FILE = "/home/vadl/catkin_ws/src/vadl-2020/slam_evaluation/gps_data/gps_odom.txt"

with open(SLAM_ODOM_FILE, "r") as f_slam, open(GPS_ODOM_FILE) as f_gps:
    
    slam_odom = f_slam.readlines()
    gps_odom = f_gps.readlines()

    for x,y in slam_odom, gps_odom:
        print(x)
        print(y)

    f_slam.close()
    f_gps.close()

