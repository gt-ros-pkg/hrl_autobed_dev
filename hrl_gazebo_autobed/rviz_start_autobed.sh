#!/bin/bash
#roslaunch ./launch/autobed_param_upload.launch  
#roslaunch ./launch/display.launch
roslaunch hrl_gazebo_autobed rviz_start.launch &
rosrun image_view image_view image:=/camera/rgb/image_color
