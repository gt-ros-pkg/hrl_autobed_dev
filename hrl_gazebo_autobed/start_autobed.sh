#!/bin/bash

roslaunch ./launch/autobed_param_upload.launch &
sleep 6 

gazebo ./autobed.world &
sleep 6 

roslaunch ./launch/autobed_default_controllers.launch

