# ECE484_Final_Project
The problem is to plan and follow a trajectory from point A to point B avoiding obstacles with smaller errors. Planning: Determine the trajectory that GEM has to follow, this is done using sampling based planning, namely RRT* Control: Control the vehicle to track the trajectory resulted from planning using a PID controller.

# Claim
Sampling technique RRT* is used to generate waypoints that obey the non-holonomic constraints of the vehicle
We have used Dubins RRT* to solve the problem of constraints
Have implemented A-star with a polynomial

To run the files:


$ cd ~/Apollo_ws/
$ catkin_make

$ source devel/setup.bash
$ roslaunch basic_launch gnss_sensor_init.launch

$ source devel/setup.bash
$ roslaunch basic_launch gnss_visualization.launch

$ source devel/setup.bash
$ roslaunch basic_launch dbw_joystick.launch

$ source devel/setup.bash
$ rosrun gem_gnss rrt_star_dubins_online.py

Ctrl + C -- To get the most recent state

$ source devel/setup.bash
$ rosrun gem_gnss gem_gnss_pp_tracker_pid.py


# Final Project Video
https://youtu.be/eCgasHrPLh4
