# Dronekit Tests

[Dronekit](https://github.com/dronekit/dronekit-python) is an API that allows developers to communicate with vehicles over [MAVLink](https://mavlink.io/en/), a messaging protocol that is used on the [Pixhawk](https://pixhawk.org/) flight controller. The Pixhawk is an amazing flight controller to use for autonomous flight, but there is still a lot of complexity in setting up communication between the Pixhawk and an onboard processor, such as the Raspberry Pi. Dronekit makes it very simple to connect and send various navigation commands to the Pixhawk. This folder is just a bunch of scripts that test various Dronekit functions. To learn how to set up Dronekit on a Raspberry Pi, check out these [instructions](../docs/raspberry_pi_ubuntu_mate_setup.md).

Suggestion of the order to run tests:

* [simple_vehicle_state.py](simple_vehicle_state.py): Test connecting to the vehicle and printing vehicle parameters.
* [takeoff.py](takeoff.py): The UAV will takeoff, hover, and land in the same position.
* [simple_goto.py](simple_goto.py) / [simple_goto_body.py](simple_goto_body.py): The UAV will takeoff, move in a direction, and land. 
* [takeoff_realsense.py](takeoff_realsense.py): This is only for brave souls. There is a lot that must be done to achieve SLAM flight, as discussed [here](../docs/achieving_slam_flight.md). Do not attempt this until you are reasonably confident the drone will do what you expect. 

Dronekit SITL can be used to test Dronekit scripts on your local computer before porting to the drone. It has limited functionality but can be very useful to avoid crashing your drone every time you change one line of code. Check [takeoff.py](takeoff.py) for an example of how to connect to SITL. After connecting, commands are sent with the same interface as a real Pixhawk.

The Dronekit functions necessary to this project have been modularized and placed in [dronekit_utils.py](../utils/dronekit_utils.py). Check there for detailed documentation on individual commands. 