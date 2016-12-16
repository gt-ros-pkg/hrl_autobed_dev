#!/bin/bash

~/git/hrl_autobed_dev/autobed_web/scripts/autobed_kill.sh

source ~/.bashrc

roslaunch autobed_web autobed_web.launch &
sleep 10
roslaunch autobed_engine autobed_engine.launch
