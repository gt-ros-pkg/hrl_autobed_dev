#!/usr/bin/env python
import sys
import serial
import numpy as np

import roslib; roslib.load_manifest('hrl_autobed_dev')
import rospy, rosparam
import serial_driver
import sharp_prox_driver

from std_msgs.msg import Bool
from std_msgs.msg import Float32
from hrl_msgs.msg import FloatArrayBare
from geometry_msgs.msg import Transform, Vector3, Quaternion

from m3skin_ros.srv import None_TransformArray, None_TransformArrayResponse
from m3skin_ros.srv import None_String, None_StringResponse

from numpy import sin, linspace, pi
from pylab import plot, show, title, xlabel, ylabel, subplot
from scipy import fft, arange

"""This is the maximum error allowed in our control system."""
ERROR_OFFSET = [5, 2, 5]#degrees, centimeters , degrees
"""List of positive movements"""
AUTOBED_COMMANDS = [[0, 'A', 'F'], [0, 'C', 'D'], [0, 'B', 'E']]#Don't ask why this isn't in alphbetical order, its Henry Clever's boo-boo. Needs to change on the Arduino.
"""Number of Actuators"""
NUM_ACTUATORS = 3



##
#Class AutobedClient()
#gives interface to connect to the Autobed, that contains methods to run the autobed engine,
#and access the sharp IR sensors on board the bed.
#

class AutobedClient():

    def __init__(self, dev, param_file, baudrate, num_of_sensors):
        '''Autobed engine node that listens into the base selection output data array
        and feeds the same as an input to the autobed control system. Further, it listens to the sensor
        position'''
        self.dev = dev
        self.param_file = param_file
        self.baudrate = baudrate 
        self.num_of_sensors = num_of_sensors
        self.reached_destination = True * np.ones(NUM_ACTUATORS)
        self.actuator_number = 0
        #Create a serial object for the Autobed to communicate with sensors and actuators. 
        self.autobed_sender = serial.Serial(dev, baudrate = baudrate)
        #Create a proximity sensor object 
        self.prox_driver = sharp_prox_driver.ProximitySensorDriver(int(num_of_sensors), param_file = self.param_file, dev = self.dev,  baudrate = self.baudrate)
        # Input to the control system.
        self.autobed_u = self.positions_in_autobed_units((self.prox_driver.get_sensor_data()[-1])[:NUM_ACTUATORS])
        #Start a publisher to publish autobed status and error 
        self.abdout0 = rospy.Publisher("/abdout0", FloatArrayBare)
        self.abdstatus0 = rospy.Publisher("/abdstatus0", Bool)
        #Start a subscriber to run the autobed engine when we get a command
        rospy.Subscriber("/abdin0", FloatArrayBare, self.autobed_engine_callback)
        #Let the sensors warm up
        print 'Initializing Autobed 1.5 ...'
        rospy.sleep(1.)

    def positions_in_autobed_units(self, distances):
        ''' Accepts position of the obstacle which is placed at 
        4.92 cm(I tried to keep it 5 cm away) from sensor at 0.0 degrees 
        and is at 18.37 cm away from sensor at 74.64 degrees(which is maximum),
        For the foot sensor, the sensor shows a reading of about 10.70cm for maximum angle of 61 degrees and value of 15.10 for an angle of 0 degrees.
        and returns value of the head tilt in degrees'''
    
        if distances[0] <= 12.25:
            distances[0] = (3.197*distances[0] - 12.56)
        elif distances[0] > 12.25 and distances[0] <= 19.5:
            distances[0] = (3.16*distances[0] - 12.11)
        elif distances[0] > 19.5 and distances[0] <= 22.4:
            distances[0] = (3.61*distances[0] - 21.02)
        elif distances[0] > 22.4 and distances[0] <= 27.5:
            distances[0] = (3.96*distances[0] - 28.70)
        else:
            distances[0] = 80
            
        
        if distances[2] >= 23.00:
            distances[2] = 0
        elif distances[2] >= 20.4 and distances[2] < 23.00:
            distances[2] = -5.176*distances[2] + 120.56
        elif distances[2] >= 17.54 and distances[2] < 20.4:
            distances[2] = -5.02*distances[2] + 117.459
        elif distances[2] >= 15.23 and distances[2] <17.54:
            distances[2] = -5.273*distances[2] + 121.81
        elif distances[2] >= 13.87 and distances[2] < 15.23:
            distances[2] = -5.664*distances[2] + 127.77
        else:
            distances[2] = -5.664*distances[2] + 127.77
        return distances

    def autobed_engine_callback(self, data):
        ''' Accepts incoming position values from the base selection algorithm and assigns it to a
        global variable. This variable is then used to guide the autobed to the desired position 
        using the engine'''

        #rospy.loginfo('[Autobed Engine Listener Callback] I heard the message: {}'.format(data.data)) 
        self.autobed_u = np.asarray(data.data)
        #We threshold the incoming data
        u_thresh = np.array([80.0, 30.0, 50.0])
        l_thresh = np.array([1.0, 9.0, 1.0])
        self.autobed_u[self.autobed_u > u_thresh] = u_thresh[self.autobed_u > u_thresh]
        self.autobed_u[self.autobed_u < l_thresh] = l_thresh[self.autobed_u < l_thresh]
        #Make reached_destination boolean flase for all the actuators on the bed.
        self.reached_destination = False * np.ones(NUM_ACTUATORS)
        self.actuator_number = 0

    def run(self): 
        rate = rospy.Rate(5) #5 Hz
        self.actuator_number = 0 #Variable that denotes what actuator is presently being controlled
        '''Initialize the autobed input to the current sensor values, so that the autobed doesn't move unless commanded'''
        autobed_u =   np.asarray(self.positions_in_autobed_units((self.prox_driver.get_sensor_data()[-1])[:NUM_ACTUATORS]))

        while not rospy.is_shutdown(): 
            #Compute error vector
            autobed_error = np.asarray(self.autobed_u - self.positions_in_autobed_units((self.prox_driver.get_sensor_data()[-1])[:NUM_ACTUATORS]))
            rospy.loginfo('autobed_u = {}, sensor_data= {}, error = {}, reached_destination = {}, self.actuator_number = {}'.format(self.autobed_u, self.positions_in_autobed_units((self.prox_driver.get_sensor_data()[-1])[:NUM_ACTUATORS]), autobed_error, self.reached_destination, self.actuator_number))
            #Publish present Autobed sensor readings 
            self.abdout0.publish(self.positions_in_autobed_units((self.prox_driver.get_sensor_data()[-1])[:NUM_ACTUATORS]))
            if self.reached_destination.all() == False:
                '''If the error is greater than some allowed offset, then we actuate the motors to get closer to desired position'''
                if self.actuator_number < (NUM_ACTUATORS):
                    if abs(autobed_error[self.actuator_number]) > ERROR_OFFSET[self.actuator_number]:
                        self.autobed_sender.write(AUTOBED_COMMANDS[self.actuator_number][int(autobed_error[self.actuator_number]/abs(autobed_error[self.actuator_number]))]) 
                        self.reached_destination[self.actuator_number] = False
                    else:
                        self.reached_destination[self.actuator_number] = True
                        '''We have reached destination for actuator self.actuator_number. Upgrade the actuation count'''
                        self.actuator_number += 1

            '''If we have reached the destination position at all the actuators, then publish the error and a boolean that says we have reached'''
            if self.reached_destination.all() == True:
                self.abdstatus0.publish(True)
            rate.sleep()



'''Runs the Autobed robot using an object of the AutobedClient class and the run method provided therein'''
if __name__ == "__main__":
    dev = sys.argv[1]
    param_file = sys.argv[2]
    baudrate = sys.argv[3]
    num_of_sensors = sys.argv[4]
    #Initialize autobed node
    rospy.init_node('autobed_engine', anonymous = True)
    autobed = AutobedClient(dev, param_file, baudrate, num_of_sensors) 
    autobed.run()



