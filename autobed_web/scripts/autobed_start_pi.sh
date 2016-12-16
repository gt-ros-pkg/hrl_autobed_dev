#!/bin/bash

~/git/hrl_autobed_dev/autobed_web/scripts/autobed_kill_pi.sh

source ~/.bashrc

sleep 3

screen -dmS autobed-pi
screen -S autobed-marvin -p 0 -X stuff "roscore
"
sleep 5
screen -S autobed-pi -X screen -t web
screen -S autobed-pi -p web -X stuff "roslaunch autobed_web autobed_web_pi.launch
"
sleep 10
screen -S autobed-pi -X screen -t engine
screen -S autobed-pi -p engine -X stuff "roslaunch autobed_engine autobed_engine.launch
"
#roslaunch autobed_web autobed_web.launch &
#sleep 10
#roslaunch autobed_engine autobed_engine.launch
