#!/bin/bash

~/ros_workspace/git/hrl_autobed_dev/autobed_web/scripts/autobed_kill_marvin.sh

source ~/.bashrc

sleep 3

#roslaunch autobed_web autobed_marvin.launch &
#~/ros_workspace/git/fsa_mat_64/bin/fsascan &
echo "Starting to run all things on marvin (in a screen)"
screen -dmS autobed-marvin
#screen -S autobed-marvin -p 0 -X stuff "roscore
#"
sleep 5
screen -S autobed-marvin -X screen -t hokuyo
screen -S autobed-marvin -p hokuyo -X stuff "roslaunch autobed_web autobed_marvin.launch
"
screen -S autobed-marvin -X screen -t fsascan
screen -S autobed-marvin -p fsascan -X stuff "~/ros_workspace/git/fsa_mat_64/bin/fsascan
"
sleep 10
screen -S autobed-marvin -X screen -t engine
screen -S autobed-marvin -p engine -X stuff "roslaunch autobed_engine autobed_engine.launch
"

#echo "Now will start running the things on the raspberry pi. This will require you to enter the password for pi"
#ssh pi@axiom "~/git/hrl_autobed_dev/autobed_web/scripts/autobed_start_pi.sh"
#echo "Everything is now hopefully up! Things are all running in a screen on the respective machine. Attach to the screen if you'd like to see it"

# 'screen -dmS autobed-pi; screen -S autobed-pi -p 0 -X stuff "~/git/hrl_autobed_dev/autobed_web/scripts/autobed_start_pi.sh

