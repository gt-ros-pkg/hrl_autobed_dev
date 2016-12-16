#!/bin/bash

echo "Now killing all ros and autobed things so we can start fresh."
pkill -9 ros
killall -9 roscore
killall -9 rosmaster
sleep 1
killall -9 hokuyo
killall -9 python
killall -9 fsascan
killall -9 hokuyo_node
killall -9 hokuyo_interpreter
echo "Finished killing all ros and autobed things."

