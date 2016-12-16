#!/bin/bash

echo "Now killing all ros and autobed things so we can start fresh."

rosnode kill /autobed/height_hokuyo_node
rosnode kill /autobed/hokuyo_interpreter_node
rosnode kill /fsamat
sleep 1
pkill -9 ros
killall -9 roscore
killall -9 rosmaster
sleep 1
killall -9 hokuyo
killall -9 python
killall -9 fsascan
killall -9 hokuyo_node
killall -9 hokuyo_interpreter
screen -S autobed-marvin -X quit
echo "Finished killing all ros and autobed things."

