#!/bin/bash

echo "***************** Beginning trial for file: $1 **********************"
echo "################ BOOTING MOTION CAPTURE SENSORS #####################"
#rosrun ros_vrpn_client ros_vrpn_client __name:=mat_o _vrpn_server_ip:=128.61.143.164 &
rosrun ros_vrpn_client ros_vrpn_client __name:=head_o _vrpn_server_ip:=128.61.143.164 &
rosrun ros_vrpn_client ros_vrpn_client __name:=l_hand_o _vrpn_server_ip:=128.61.143.164 &
rosrun ros_vrpn_client ros_vrpn_client __name:=r_hand_o _vrpn_server_ip:=128.61.143.164 &
rosrun ros_vrpn_client ros_vrpn_client __name:=l_elbow_o _vrpn_server_ip:=128.61.143.164 &
rosrun ros_vrpn_client ros_vrpn_client __name:=r_elbow_o _vrpn_server_ip:=128.61.143.164 &
rosrun ros_vrpn_client ros_vrpn_client __name:=l_knee_o _vrpn_server_ip:=128.61.143.164 &
rosrun ros_vrpn_client ros_vrpn_client __name:=r_knee_o _vrpn_server_ip:=128.61.143.164 &
rosrun ros_vrpn_client ros_vrpn_client __name:=l_ankle_o _vrpn_server_ip:=128.61.143.164 &
rosrun ros_vrpn_client ros_vrpn_client __name:=r_ankle_o _vrpn_server_ip:=128.61.143.164 &
rosrun ros_vrpn_client ros_vrpn_client __name:=torso_o _vrpn_server_ip:=128.61.143.164 &
echo "####################################################################"
echo "####################################################################"
echo "BOOTED SENSORS. WILL BEGIN RECORDING IN 10 SECONDS"
sleep 5
echo "10........."
sleep 1
echo "9........."
sleep 1
echo "8........"
sleep 1
echo "7......."
sleep 1
echo "6......"
sleep 1
echo "5....."
sleep 1
echo "4...."
sleep 1
echo "3..."
sleep 1
echo "2.."
sleep 1
echo "1."
sleep 1
echo "***************** Starting Recording Be Ready ***********************"

rosbag record -O ./bagfiles/$1 /fsascan /camera/rgb/image_color /head_o/pose /mat_o/pose /l_hand_o/pose /r_hand_o/pose /l_knee_o/pose /r_knee_o/pose /l_ankle_o/pose /r_ankle_o/pose /abdout0 /torso_o/pose /l_elbow_o/pose /r_elbow_o/pose 

