#!/bin/bash
rosrun autobed_pose_estimator bag_to_p.py database/supine_2.p &
rosbag play ../autobed_physical_trainer/bagfiles/supine_2.bag &
sleep 500
rosnode kill -a
echo "DONE WITH SUPINE POSES"
#rosrun autobed_pose_estimator bag_to_p.py database/right_lateral_1.p &
#rosbag play ../autobed_physical_trainer/bagfiles/rightlateral_1.bag &
#sleep 600
#rosnode kill -a
#echo "DONE WITH RIGHT LATERAL POSES"
rosrun autobed_pose_estimator bag_to_p.py database/left_lateral_2.p &
rosbag play ../autobed_physical_trainer/bagfiles/leftlateral_1.bag &
sleep 400
rosnode kill -a
echo "DONE WITH LEFT LATERAL POSES"





