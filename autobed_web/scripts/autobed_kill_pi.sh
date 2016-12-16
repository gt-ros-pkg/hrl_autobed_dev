#!/bin/bash

echo "Now killing all ros and autobed things so we can start fresh."
rosnode kill /rosbridge_abd
rosnode kill /rosapi_abd
rosnode kill /roswww_abd
rosnode kill /autobed_engine
sleep 1
pkill -9 ros
killall -9 roscore
killall -9 rosmaster
sleep 1
killall -9 autobed_engine
killall -9 python
killall -9 autobed_web
killall -9 rosbridge_abd
killall -9 rosapi_abd
killall -9 roswww_abd
screen -S autobed-pi -X quit
echo "Finished killing all ros and autobed things."

