#!/bin/bash

~/git/hrl_autobed_dev/autobed_web/scripts/autobed_kill_marvin.sh

source ~/.bashrc

roscore &
ssh pi@axiom 'cd ~/git/hrl_autobed_dev/autobed_web/scripts/; ./autobed_start_pi.sh' &

sleep 5

roslaunch autobed_web autobed_marvin.launch &
~/ros_workspace/git//fsa_mat_64/bin/fsascan &
