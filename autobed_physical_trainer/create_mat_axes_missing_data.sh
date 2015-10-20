#!/bin/bash
#Usage: ./convert_bag_to_p.sh subject_n
#subject_n is an argument that represents the number of the subject you are 
#converting data for.
#For instance, if we are looking at subject 1, I will run:
#./convert_bag_to_p.sh subject_1

mkdir dataset/$1
echo "NEW FOLDER CREATED FOR NEW SUBJECT IN DATASET"
rosbag play ./bagfiles/$1/home_sup.bag &
sleep 10
python src/mat_axes_generator_missing_data.py ./dataset/$1/ &
sleep 100
echo "KILLING NODE"
rosnode kill mat_to_pkl
sleep 20
echo "CAPTURED MAT AXES"
###############################################################################
rosbag play bagfiles/$1/salvaged_mat_o.bag &
sleep 5
python src/mat_axes_generator_missing_data.py ./dataset/$1/ &
sleep 100
echo "DONE WITH CONVERTING POSES FOR HOME POSITION"
echo "KILLING NODE"
rosnode kill mat_to_pkl
sleep 20
echo "RECOVERED MAT AXES"
###############################################################################
