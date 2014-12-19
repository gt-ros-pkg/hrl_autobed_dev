#!/bin/bash
echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
echo "Beginning of AutoBed Start Script"
date
echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
source /opt/ros/fuerte/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/pi/git

sleep 1
roslaunch /home/pi/git/hrl_autobed_dev/autobed_web/launch/autobed_web.launch &
sleep 5 
roslaunch /home/pi/git/hrl_autobed_dev/autobed_engine/launch/autobed_engine.launch &
