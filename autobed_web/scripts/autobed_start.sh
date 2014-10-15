#!/bin/bash

source /opt/ros/fuerte/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/mycroft/git/
export ROS_MASTER_URI=http://localhost:11312
roscore -p 11312 &
sleep 10
roslaunch autobed_web autobed_web.launch &
sleep 10
roslaunch autobed_engine autobed_engine.launch
