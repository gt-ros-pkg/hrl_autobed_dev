#!/usr/bin/env python
import sys
import serial
import numpy as np

import roslib; roslib.load_manifest('autobed_engine')
import rospy
import serial_driver
import sharp_prox_driver
import cPickle as pkl
from hrl_lib.util import save_pickle, load_pickle


from std_msgs.msg import Bool, Float32, String, Int16
from hrl_msgs.msg import FloatArrayBare, StringArray
from geometry_msgs.msg import Transform, Vector3, Quaternion

from m3skin_ros.srv import None_TransformArray, None_TransformArrayResponse, None_String, None_StringResponse
from autobed_engine.srv import *

"""List of positive movements"""
AUTOBED_COMMANDS = [[0, 'F', 'A'], [0, 'D', 'B'], [0, 'E', 'C']]#Don't ask why this isn't in alphbetical order, its Henry Clever's boo-boo. Needs to change on the Arduino.
"""Number of Actuators"""
NUM_ACTUATORS = 3
""" Basic Differential commands to the Autobed via GUI"""
CMDS = {'headUP': 'F', 'headDN': 'A', 'bedUP':'D', 'bedDN':'B', 'legsUP':'E', 'legsDN':'C'}

##
#Class AutobedClient()
#gives interface to connect to the Autobed, that contains methods to run the autobed engine,
#and access the sharp IR sensors on board the bed.
#

class AutobedClient():
    """  Gives interface to connect to the Autobed, that contains methods to run the autobed engine,
         and access the sharp IR sensors on board the bed."""

    def __init__(self, dev, autobed_config_file, param_file, baudrate, num_of_sensors):
        '''Autobed engine node that listens into the base selection output data array
        and feeds the same as an input to the autobed control system. Further, it listens to the sensor
        position'''
        self.dev = dev
        self.autobed_config_file = autobed_config_file
        self.param_file = param_file
        self.baudrate = baudrate
        self.num_of_sensors = num_of_sensors
        #Create a serial object for the Autobed to communicate with sensors and actuators.
        self.autobed_sender = serial.Serial(dev, baudrate = baudrate)
        # Input to the control system.

        self.abdout1 = rospy.Publisher("/abdout1", StringArray, latch=True)
        #Start a subscriber that takes a differential input and just relays it to the autobed.
        rospy.Subscriber("/abdin1", String, self.differential_control_callback)

        print 'Initializing Autobed GUI...'
        rospy.sleep(1.)

    def differential_control_callback(self, data):
        ''' Accepts incoming differential control values and simply relays them to the Autobed.
        This mode is used when Henry wants to control the autobed manually even if no sensors are present'''
        autobed_config_data = load_pickle(self.autobed_config_file) 
        if data.data in CMDS: 
            self.autobed_sender.write(CMDS[data.data])
        else:
            rospy.loginfo('Error: Wrong command sent to autobed')



    def run(self):
        rate = rospy.Rate(5) #5 Hz
        while not rospy.is_shutdown():
            rate.sleep()

'''Runs the Autobed robot using an object of the AutobedClient class and the run method provided therein'''
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="ROS Interface for AutoBed")
    parser.add_argument("serial_device", type=str, help="The serial device for the AutoBed Arduino serial connection.")
    parser.add_argument("baudrate", type=int, help="AutoBed Serial Baudrate")
    parser.add_argument("sensor_param_file", type=str, help="The paramter file describing the autobed sensors")
    parser.add_argument("number_of_sensors", type=int, help="Number of sensors on the AutoBed", default=4)
    parser.add_argument("autobed_config_file", type=str, help="Configuration file fo the AutoBed")
    args = parser.parse_args(rospy.myargv()[1:])
    #Initialize autobed node
    rospy.init_node('autobed_engine', anonymous=True)
    autobed = AutobedClient(args.serial_device,
                            args.autobed_config_file,
                            args.sensor_param_file,
                            args.baudrate,
                            args.number_of_sensors)
    autobed.run()
