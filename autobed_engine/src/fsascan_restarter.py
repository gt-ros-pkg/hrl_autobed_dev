#!/usr/bin/env python

import math, numpy as np
import sys, time, random

NAME = 'utm_python_listener'
import roslib; roslib.load_manifest('autobed_engine')
import rospy
import subprocess
from sensor_msgs.msg import LaserScan
import threading
from std_msgs.msg import Float32
from hrl_msgs.msg import FloatArrayBare

import copy

import numpy as np

class FSAScanRestarter():
    ''' This class has the data of a laser scan.
    '''
    def __init__(self):
        self.frame_lock = threading.RLock()
        self.lockout = False
        rospy.Subscriber("/fsascan", FloatArrayBare, self.fsascan_cb, queue_size=1)

    def fsascan_cb(self, msg):
        with self.frame_lock:
            if not self.lockout:
                if not msg.data:
                    self.lockout = True
                    print 'The fsa scan (pressure mat) gave an empty list. Probably something has gone wrong.'
                    print 'I will try restarting it!'
                    subprocess.Popen(["rosnode", "kill", "fsamat"])
                    rospy.sleep(2)
                    subprocess.Popen(["screen", "-S", "autobed-marvin", "-p", "fsascan", "-X", "stuff",
                                      "~/ros_workspace/git/fsa_mat_64/bin/fsascan\r"])
                    rospy.sleep(2)
                    self.lockout = False


if __name__ == '__main__':
    rospy.init_node('fsascan_restarter')
    res = FSAScanRestarter()
    rospy.spin()




