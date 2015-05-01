#!/bin/bash
rosrun autobed_physical_trainer bag_to_p.py dataset/ari_pose_01052015_partial.p &
rosbag play bagfiles/ari_01052015_partial.bag &
sleep 160 
rosnode kill -a
echo "DONE WITH CONVERTING POSES"

