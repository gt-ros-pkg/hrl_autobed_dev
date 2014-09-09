#!/usr/bin/env python

import sys
import serial
import numpy as np

import roslib; roslib.load_manifest('hrl_autobed_dev')
import rospy, rosparam
import serial_driver
import sharp_prox_driver

from std_msgs.msg import Float32
from hrl_msgs.msg import FloatArrayBare
from geometry_msgs.msg import Transform, Vector3, Quaternion

from m3skin_ros.srv import None_TransformArray, None_TransformArrayResponse
from m3skin_ros.srv import None_String, None_StringResponse

from numpy import sin, linspace, pi
from pylab import plot, show, title, xlabel, ylabel, subplot
from scipy import fft, arange

"""This is the maximum error allowed in our control system."""
ERROR_OFFSET = [5, 2, 1000]#degrees, centimeters , degrees
"""List of positive movements"""
AUTOBED_COMMANDS = [[0, 'A', 'F'], [0, 'C', 'D'], [0, 'B', 'E']]
"""Number of Actuators"""
NUM_ACTUATORS = 3
""" Input to the control system."""
global autobed_u
autobed_u = np.zeros(NUM_ACTUATORS) 

def positions_in_autobed_units(distances):
    ''' Accepts position of the obstacle which is placed at 
    4.92 cm(I tried to keep it 5 cm away) from sensor at 0.0 degrees and is at 18.37 cm away from sensor at 74.64 degrees(which is maximum), and returns value of the head tilt in degrees'''
    distances[0] = (3.4294*distances[0] - 16.87240)
    return distances

def autobed_engine_callback(data):
    ''' Accepts incoming position values from the base selection algorithm and assigns it to a
    global variable. This variable is then used to guide the autobed to the desired position 
    using the engine'''

    rospy.loginfo('[Autobed Engine Listener Callback] I heard the message: {}'.format(data.data)) 
    global autobed_u
    autobed_u = np.asarray(data.data)


def autobed_engine():
    '''Autobed engine node that listens into the base selection output data array
    and feeds the same as an input to the autobed control system. Further, it listens to the sensor
    position'''
    baudrate = sys.argv[3]
    global autobed_u
    #Initialize listener node to hear command
    rospy.init_node('autobed_engine', anonymous = True)
    #Sends a serial command to the Autobed
    autobed_sender = serial.Serial('/dev/ttyUSB0', baudrate = baudrate)
    #Create a proximity sensor object 
    prox_driver = sharp_prox_driver.ProximitySensorDriver(int(sys.argv[4]), param_file = sys.argv[2], dev = sys.argv[1], baudrate = baudrate)
    #Making sure that we dont travel too much before base selection is done
    autobed_u = positions_in_autobed_units((prox_driver.get_sensor_data()[-1])[:NUM_ACTUATORS])
    #Start a subscriber to run the autobed engine when we get a command
    rospy.Subscriber("/abdin0", FloatArrayBare, autobed_engine_callback)
    
    rate = rospy.Rate(5) #5 Hz
    while not rospy.is_shutdown():
        #Compute error vector
        autobed_error = np.asarray(autobed_u - positions_in_autobed_units((prox_driver.get_sensor_data()[-1])[:NUM_ACTUATORS]))
        rospy.loginfo('autobed_u = {}, sensor_data= {}, error = {}'.format(autobed_u, positions_in_autobed_units((prox_driver.get_sensor_data()[-1])[:NUM_ACTUATORS]), autobed_error))
        #Zero the error if its too small
        autobed_error[np.absolute(autobed_error) < ERROR_OFFSET] = 0.0 
             
        for i in range(NUM_ACTUATORS):
            if abs(autobed_error[i]) > 0.0:
                autobed_sender.write(AUTOBED_COMMANDS[i][int(autobed_error[i]/abs(autobed_error[i]))]) 
            else:
                rospy.loginfo('[Autobed engine] At destination with error: {}'.format(autobed_error[i]))

        rate.sleep()



if __name__ == "__main__":
    autobed_engine()

