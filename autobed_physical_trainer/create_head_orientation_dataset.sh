#!/bin/bash
#Usage: ./convert_bag_to_p.sh subject_n
#subject_n is an argument that represents the number of the subject you are 
#converting data for.
#For instance, if we are looking at subject 1, I will run:
#./convert_bag_to_p.sh subject_1

###############################################################################
python src/head_orientation_to_p.py dataset/head_orientation_dataset.p &
rosbag play bagfiles/subject_9/head_sup.bag &
sleep 120
echo "DONE WITH CONVERTING POSES FOR HOME POSITION"
###############################################################################
rosbag play bagfiles/subject_10/head_sup.bag &
sleep 120
echo "DONE WITH CONVERTING POSES FOR HOME POSITION"
###############################################################################
rosbag play bagfiles/subject_11/head_sup.bag &
sleep 120
echo "DONE WITH CONVERTING POSES FOR HOME POSITION"
###############################################################################
rosbag play bagfiles/subject_12/head_sup.bag &
sleep 120
echo "DONE WITH CONVERTING POSES FOR HOME POSITION"
###############################################################################
rosbag play bagfiles/subject_13/head_sup.bag &
sleep 120
echo "DONE WITH CONVERTING POSES FOR HOME POSITION"
###############################################################################
rosbag play bagfiles/subject_14/head_sup.bag &
sleep 120
echo "DONE WITH CONVERTING POSES FOR HOME POSITION"
###############################################################################
rosbag play bagfiles/subject_15/head_sup.bag &
sleep 120
echo "DONE WITH CONVERTING POSES FOR HOME POSITION"
###############################################################################

echo "KILLING NODE"
rosnode kill bag_to_pkl
sleep 20
echo "MOVING ON TO NEXT BAG FILE"
###############################################################################

