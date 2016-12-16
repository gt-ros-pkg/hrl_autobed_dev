#!/bin/bash

~/ros_workspace/git/hrl_autobed_dev/autobed_web/scripts/autobed_kill_marvin.sh

source ~/.bashrc



#sleep 5

#roslaunch autobed_web autobed_marvin.launch &
#~/ros_workspace/git/fsa_mat_64/bin/fsascan &

screen -dmS autobed-marvin
screen -S autobed-marvin -p 0 -X stuff "roscore
"
sleep 5
screen -S autobed-marvin -X screen -t hokuyo
screen -S autobed-marvin -p hokuyo -X stuff "roslaunch autobed_web autobed_marvin.launch
"
screen -S autobed-marvin -X screen -t fsascan
screen -S autobed-marvin -p fsascan -X stuff "~/ros_workspace/git/fsa_mat_64/bin/fsascan
"
ssh pi@axiom "~/git/hrl_autobed_dev/autobed_web/scripts/autobed_start_pi.sh"
# 'screen -dmS autobed-pi; screen -S autobed-pi -p 0 -X stuff "~/git/hrl_autobed_dev/autobed_web/scripts/autobed_start_pi.sh

