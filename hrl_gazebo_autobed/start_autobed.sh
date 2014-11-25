#!/bin/bash

roslaunch /home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/launch/autobed_param_upload.launch &
sleep 6 

gazebo /home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/autobed.world &
sleep 6 

name="new_ragdoll"
echo "=================================================="
echo "Tweaking Initial poses for Human" 
echo "=================================================="
python /home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/autobed_pose_estimator/src/model_plugin_modifier.py 16 
rm /home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/sdf/new_ragdoll/gazebo_model_plugin/ros_ragdoll_model2_plugin.cc
mv /home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/sdf/new_ragdoll/gazebo_model_plugin/ros_ragdoll_model3_plugin.cc /home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/sdf/new_ragdoll/gazebo_model_plugin/ros_ragdoll_model2_plugin.cc
rm /home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/sdf/new_ragdoll/correct_ragdoll_original.sdf
mv /home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/sdf/new_ragdoll/correct_ragdoll_temporary.sdf /home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/sdf/new_ragdoll/correct_ragdoll_original.sdf

echo "=================================================="
echo "Initializing Human"
echo "=================================================="
rm -rf /home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/sdf/new_ragdoll/gazebo_model_plugin/build 
mkdir /home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/sdf/new_ragdoll/gazebo_model_plugin/build
cd /home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/sdf/new_ragdoll/gazebo_model_plugin/build  
cmake ..
make 
cd /home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/
gzfactory spawn -f /home/yashc/fuerte_workspace/sandbox/git/hrl_autobed_dev/hrl_gazebo_autobed/sdf/new_ragdoll/correct_ragdoll_original.sdf -m $name 

sleep 6 
roslaunch ./launch/autobed_default_controllers.launch

