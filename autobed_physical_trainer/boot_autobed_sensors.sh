#!/bin/bash
roslaunch openni_launch openni.launch 
rosrun fsa_mat_64 fsascan &
rosrun ros_vrpn_client ros_vrpn_client __name:=abd_head_angle _vrpn_server_ip:=128.61.143.164 &
rosrun ros_vrpn_client ros_vrpn_client __name:=abd_leg_angle _vrpn_server_ip:=128.61.143.164 &
rosrun ros_vrpn_client ros_vrpn_client __name:=camera_o _vrpn_server_ip:=128.61.143.164 &

