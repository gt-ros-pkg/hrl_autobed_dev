#!/bin/bash
rosrun autobed_physical_trainer bag_to_p.py dataset/megan_train_01052015_full.p &
rosbag play bagfiles/megan_train_13052015_full.bag &
sleep 160 
rosnode kill -a
echo "DONE WITH CONVERTING POSES"

