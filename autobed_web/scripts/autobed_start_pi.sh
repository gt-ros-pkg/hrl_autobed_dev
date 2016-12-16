#!/bin/bash

~/git/hrl_autobed_dev/autobed_web/scripts/autobed_kill_pi.sh

source ~/.bashrc

screen -dmS autobed-pi
screen -S autobed-pi -p 0 -X stuff "roslaunch autobed_web autobed_web.launch
"
sleep 5
screen -S autobed-pi -X screen -t engine
screen -S autobed-pi -p 0 -X stuff "roslaunch autobed_engine autobed_engine.launch
"
#roslaunch autobed_web autobed_web.launch &
#sleep 10
#roslaunch autobed_engine autobed_engine.launch
