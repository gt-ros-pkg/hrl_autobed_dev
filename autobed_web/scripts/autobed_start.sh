#!/bin/bash

source /opt/ros/fuerte/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/pi/git/
export ROS_MASTER_URI=http://localhost:11311
roslaunch autobed_web autobed_web.launch &
sleep 10
roslaunch autobed_engine autobed_engine.launch
