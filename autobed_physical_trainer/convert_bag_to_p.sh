#!/bin/bash
rosrun autobed_physical_trainer bag_to_p.py dataset/yashc_2015_02_21_15_54.p &
rosbag play bagfiles/yashc_2015_02_21_15_54.bag &
sleep 174 
rosnode kill -a
echo "DONE WITH CONVERTING POSES"

