#!/bin/bash
echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
echo "Beginning of AutoBed Start Script"
date
echo ">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>"
source /opt/ros/fuerte/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/pi/git

echo "<<<<< ENV SETUP Done <<<<<<<<<"
sleep 10 

echo "<<<<< Starting Autobed Web Roslaunch <<<<"
date
roslaunch /home/pi/git/hrl_autobed_dev/autobed_web/launch/autobed_web.launch &
echo "<<<<<< Autobed Web launched <<<<<<"
#sleep 10 

#echo "<<<<< Starting Autobed Engine Roslaunch <<<<"
#date
#roslaunch /home/pi/git/hrl_autobed_dev/autobed_engine/launch/autobed_engine.launch &
#echo "<<<<< Autobed Engine launched <<<<"
