#!/bin/bash

~/git/hrl_autobed_dev/autobed_web/scripts/autobed_kill_marvin.sh

source ~/.bashrc

roscore &
ssh pi@axiom 'cd ~/git/hrl_autobed_dev/autobed_web/scripts/; ./autobed_start_pi.sh' &

sleep 5

rosparam set hokuyo_node/port /dev/sensors/hokuyo_autobed_height
roslaunch autobed_web autobed_height_hokuyo_marvin.launch
roslaunch autobed_web autobed_web.launch &
sleep 10
roslaunch autobed_engine autobed_engine.launch
