################################################################
# Demo Code Only. 
# Do not mofidy.
# Auther: Amith Ramdas Achari
################################################################


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

