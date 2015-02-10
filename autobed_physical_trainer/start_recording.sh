#!/bin/bash

echo "***************** Beginning trial for file: $1 **********************"
roslaunch openni_launch openni.launch &
#rosrun fsa_mat_64 fsascan

sleep 10
echo "3.."
sleep 1
echo "2."
sleep 1
echo "1."
sleep 1
echo "***************** Starting Recording Be Ready ***********************"

rosbag record -O ./bagfiles/$1 /camera/rgb/image_color/compressed 


