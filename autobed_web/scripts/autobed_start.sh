#!/bin/bash
echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
echo "Beginning of AutoBed Start Script"
date
echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
source /opt/ros/fuerte/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/pi/git

echo "<<<<< ENV SETUP Done <<<<<<<<<"
date
sleep 5

echo "<<<<< Starting Autobed Web Roslaunch <<<<"
roslaunch /home/pi/git/hrl_autobed_dev/autobed_web/launch/autobed_web.launch &
echo "<<<<<< Autobed Web launched <<<<<<"
date
sleep 5 

echo "<<<<< Starting Autobed Engine Roslaunch <<<<"
roslaunch /home/pi/git/hrl_autobed_dev/autobed_engine/launch/autobed_engine.launch &
echo "<<<<< Autobed Engine launched <<<<"
