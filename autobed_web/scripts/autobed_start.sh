#!/bin/bash
echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
echo "Beginning of AutoBed Start Script"
date
source /opt/ros/fuerte/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/pi/git
echo "====== Starting Autobed Web Roslaunch ======="
roslaunch /home/pi/git/hrl_autobed_dev/autobed_web/launch/autobed_web.launch &
echo "<<<<<< End of AutoBed Start Script <<<<<<"
