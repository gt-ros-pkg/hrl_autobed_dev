#!/bin/bash

roslaunch ./launch/autobed_param_upload.launch &
sleep 6 

gazebo ./autobed.world &
sleep 6 

echo "=================================================="
echo "Initializing Human"
echo "=================================================="
gzfactory spawn -f /home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/sdf/new_ragdoll/correct_ragdoll_original.sdf -m new_ragdoll -x 2.10 -y 0 -z 0.8 -R 1.570 -P 0 -Y 3.14

sleep 6 
roslaunch ./launch/autobed_default_controllers.launch

