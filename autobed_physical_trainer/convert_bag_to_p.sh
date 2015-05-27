#!/bin/bash
rosrun autobed_physical_trainer bag_to_p.py dataset/all_15052015_sup_left_right_partial.p &
#rosbag play bagfiles/yash_14052015_supine.bag &
#rosbag play bagfiles/yash_14052015_left_lateral.bag &
#rosbag play bagfiles/yash_14052015_right_lateral.bag &
#rosbag play bagfiles/megan_15052015_supine.bag &
#rosbag play bagfiles/megan_15052015_left_lateral.bag &
#rosbag play bagfiles/megan_15052015_right_lateral.bag &
#rosbag play bagfiles/daehyung_15052015_supine.bag &
#rosbag play bagfiles/daehyung_15052015_left_lateral.bag &
#rosbag play bagfiles/daehyung_15052015_right_lateral.bag &
#rosbag play bagfiles/tapo_15052015_supine.bag &
#rosbag play bagfiles/tapo_15052015_left_lateral.bag &
rosbag play bagfiles/tapo_15052015_right_lateral.bag &

sleep 80 
rosnode kill -a
echo "DONE WITH CONVERTING POSES"

