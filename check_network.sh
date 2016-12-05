#!/bin/bash 

#  This script checks to see if a particular network connection is available.
#  If it is not, it attempts to reset the connection
#  TO USE, copy the following into chron:
#  */5 * * * * /home/pi/check_network.sh >> /home/pi/log/check_network.log 2>&1
#  This will run this script every 5 minutes, and log the results to /home/pi/log/check_network.log

#  CHANGE: The IP to check (below) may be changed from 8.8.8.8 to a local router or machine
#  CHANGE: The network interface to reset (wlan0 below) - This may be different on different systems

date  
if /bin/ping -q -c 21 8.8.8.8 >> /dev/null; then 
    echo "[Check Network] Pinged router Successfully" 
    logger -i "[Check Network] Pinged Router Successfully" 
else  
    set -x  
    logger -is "[Check Network] Failed to ping router. Restarted wlan0" 
    /etc/ifplugd/ifplugd.action wlan0 down 
    sleep 15 
    /etc/ifplugd/ifplugd.action wlan0 up 
    sleep 45 
    if /sbin/ifconfig wlan | grep -q "inet addr:" ; then 
        logger -is "[Check Network] Wireless Connection Successfully Reestablished" 
    else 
        logger -is "[Check Network] Failed to Reestablish Wireless Connection. Better luck next time." 
    fi 
fi
