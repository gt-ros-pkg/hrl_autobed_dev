#!/bin/bash

~/ros_workspace/git/hrl_autobed_dev/autobed_web/scripts/autobed_kill_marvin.sh

source ~/.bashrc

roscore &

sleep 5

roslaunch autobed_web autobed_marvin.launch &
~/ros_workspace/git//fsa_mat_64/bin/fsascan &

ssh pi@axiom 'screen -dmS autobed-pi; screen -S autobed-pi -p 0 -X stuff ~/git/hrl_autobed_dev/autobed_web/scripts/autobed_start_pi.sh\012'
