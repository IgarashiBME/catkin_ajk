#!/bin/bash
source /opt/ros/kinetic/setup.bash
source ~/catkin_ajk/devel/setup.bash
gnome-terminal -e "roslaunch ubx_analyzer navpvt.launch"
