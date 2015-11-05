#!/bin/bash
#Usage: ./convert_bag_to_p.sh subject_n
#subject_n is an argument that represents the number of the subject you are 
#converting data for.
#For instance, if we are looking at subject 1, I will run:
#./convert_bag_to_p.sh subject_1

mkdir dataset_new_format/$1
echo "NEW FOLDER CREATED FOR NEW SUBJECT IN DATASET"
rosbag play ./bagfiles/$1/mat_o.bag &
sleep 5
python src/capture_mat_axes.py ./dataset_new_format/$1/ &
sleep 30
echo "KILLING NODE"
rosnode kill mat_to_pkl
sleep 20
echo "CAPTURED MAT AXES"
##############################################################################
rosrun autobed_physical_trainer bag_to_p.py dataset_new_format/$1/home_sup.p &
rosbag play bagfiles/$1/home_sup.bag &
sleep 40
echo "DONE WITH CONVERTING POSES FOR HOME POSITION"
echo "KILLING NODE"
rosnode kill bag_to_pkl
sleep 20
echo "MOVING ON TO NEXT BAG FILE"
##############################################################################
#rosrun autobed_physical_trainer bag_to_p.py dataset_new_format/$1/head_sup.p &
#rosbag play bagfiles/$1/head_sup.bag &
#sleep 140
#echo "DONE WITH CONVERTING POSES FOR HOME POSITION"
#echo "KILLING NODE"
#rosnode kill bag_to_pkl
#sleep 20
#echo "MOVING ON TO NEXT BAG FILE"
###############################################################################
rosrun autobed_physical_trainer bag_to_p.py dataset_new_format/$1/LH_sup.p &
rosbag play bagfiles/$1/LH1.bag &
sleep 120
echo "DONE WITH CONVERTING POSES FOR LEFT HAND POSITION"
echo "MOVING ON TO NEXT BAG FILE"
################################################################################
rosbag play bagfiles/$1/LH2.bag &
sleep 140
echo "DONE WITH CONVERTING POSES FOR LEFT HAND 2 POSITION"
echo "MOVING ON TO NEXT BAG FILE"
################################################################################
rosbag play bagfiles/$1/LH3.bag &
sleep 140
echo "DONE WITH CONVERTING POSES FOR LEFT HAND 3 POSITION"
echo "MOVING ON TO NEXT BAG FILE"
echo "KILLING NODE"
rosnode kill bag_to_pkl
sleep 20
echo "MOVING ON TO NEXT BAG FILE"

################################################################################
rosrun autobed_physical_trainer bag_to_p.py dataset_new_format/$1/RH_sup.p &
rosbag play bagfiles/$1/RH1.bag &
sleep 120
echo "DONE WITH CONVERTING POSES FOR RIGHT HAND POSITION"
echo "MOVING ON TO NEXT BAG FILE"
################################################################################
rosbag play bagfiles/$1/RH2.bag &
sleep 140
echo "DONE WITH CONVERTING POSES FOR LEFT HAND 2 POSITION"
echo "MOVING ON TO NEXT BAG FILE"
################################################################################
rosbag play bagfiles/$1/RH3.bag &
sleep 140
echo "DONE WITH CONVERTING POSES FOR LEFT HAND 3 POSITION"
echo "MOVING ON TO NEXT BAG FILE"
echo "KILLING NODE"
rosnode kill bag_to_pkl
sleep 20
echo "MOVING ON TO NEXT BAG FILE"
################################################################################
rosrun autobed_physical_trainer bag_to_p.py dataset_new_format/$1/LL_sup.p &
rosbag play bagfiles/$1/LL.bag &
sleep 120
echo "DONE WITH CONVERTING POSES FOR RIGHT HAND POSITION"
echo "MOVING ON TO NEXT BAG FILE"
################################################################################
echo "KILLING NODE"
rosnode kill bag_to_pkl
sleep 20
echo "MOVING ON TO NEXT BAG FILE"
###############################################################################
rosrun autobed_physical_trainer bag_to_p.py dataset_new_format/$1/RL_sup.p &
rosbag play bagfiles/$1/RL.bag &
sleep 120
echo "DONE WITH CONVERTING POSES FOR RIGHT HAND POSITION"
echo "MOVING ON TO NEXT BAG FILE"
##############################################################################
echo "DONE WITH CONVERTING POSES FOR RIGHT LEG POSITION"
echo "KILLING NODE"
rosnode kill bag_to_pkl
sleep 20
echo "END OF CONVERSION FOR SLEEPING POSES"
##############################################################################
##############################################################################
#python src/pkl_to_dataset.py --path=./dataset_new_format/$1/ &
echo "ADDED NEW SUBJECT TO THE FINAL DATASET" 
