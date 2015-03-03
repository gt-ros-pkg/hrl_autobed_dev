#!/bin/bash
roslaunch ./launch/autobed_param_upload.launch  

gazebo ./autobed.world &

roslaunch ./launch/autobed_default_controllers.launch 

sleep 30 
#kill -9 `ps aux | grep ros | awk "{print $2}"`  
sleep 1  

#kill -9 `ps aux | grep crona | awk "{print $2}"` 
sleep 1 

#kill -9 `ps aux | grep gz | awk "{print $2}"` 
sleep 1 

#kill -9 `ps aux | grep gazebo | awk "{print $2}"` 
sleep 1 

#kill -9 `ps aux | grep autobed | awk "{print $2}"` & 

sleep 10 
#kill -9 `ps aux | grep autobed | awk "{print $2}"`  
