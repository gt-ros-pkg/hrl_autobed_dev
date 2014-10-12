#!/usr/bin/env python
import sys
import serial
import numpy as np

import roslib; roslib.load_manifest('autobed_engine')
import rospy
import serial_driver
import sharp_prox_driver
import cPickle as pkl
roslib.load_manifest('hrl_lib')
from hrl_lib.util import save_pickle, load_pickle


from std_msgs.msg import Bool, Float32, String, Int16
from hrl_msgs.msg import FloatArrayBare, StringArray
from geometry_msgs.msg import Transform, Vector3, Quaternion

from m3skin_ros.srv import None_TransformArray, None_TransformArrayResponse, None_String, None_StringResponse
from autobed_engine.srv import *

#This is the maximum error allowed in our control system.
ERROR_OFFSET = [5, 2, 5] #[degrees, centimeters , degrees]

#List of positive movements
#Don't ask why this isn't in alphbetical order, its Henry Clever's boo-boo.
#Needs to change on the Arduino.
AUTOBED_COMMANDS = [[0, 'A', 'F'], [0, 'C', 'D'], [0, 'B', 'E']]

"""Number of Actuators"""
NUM_ACTUATORS = 3

CMDS = ['headUP', 'headDN', 'bedUP', 'bedDN', 'legsUP', 'legsDN']

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
        self.reached_destination = True * np.ones(NUM_ACTUATORS)
        self.actuator_number = 0
        #Create a serial object for the Autobed to communicate with sensors and actuators.
        self.autobed_sender = serial.Serial(dev, baudrate = baudrate)
        #Create a proximity sensor object
        self.prox_driver = sharp_prox_driver.ProximitySensorDriver(int(num_of_sensors),
                                                                   param_file = self.param_file,
                                                                   dev = self.dev,
                                                                   baudrate = self.baudrate)
        # Input to the control system.
        self.autobed_u = self.positions_in_autobed_units((self.prox_driver.get_sensor_data()[-1])[:NUM_ACTUATORS])
        #Start a publisher to publish autobed status and error
        self.abdout0 = rospy.Publisher("/abdout0", FloatArrayBare)
        self.abdstatus0 = rospy.Publisher("/abdstatus0", Bool)

        self.abdout1 = rospy.Publisher("/abdout1", StringArray)
        #Start a subscriber to run the autobed engine when we get a command
        rospy.Subscriber("/abdin0", FloatArrayBare, self.autobed_engine_callback)
        #Start a subscriber that takes a differential input and just relays it to the autobed.
        rospy.Subscriber("/abdin1", String, self.differential_control_callback)
        #Set up the service that stores present Autobed configuration in the yaml file
        rospy.Service('update_autobed_config', add_bed_config, self.update_autobed_configuration)

        try:
            init_autobed_config_data = load_pickle(self.autobed_config_file)
            self.abdout1.publish(init_autobed_config_data.keys())
        except:
            init_autobed_config_data = {'headUP': 'A',
                                        'headDN': 'F',
                                        'bedUP': 'C',
                                        'bedDN': 'D',
                                        'legsUP': 'B',
                                        'legsDN': 'E'}
            save_pickle(init_autobed_config_data, self.autobed_config_file)
        #Let the sensors warm up
        print 'Initializing Autobed 1.5 ...'
        rospy.sleep(1.)

    def positions_in_autobed_units(self, distances):
        '''  Accepts position of the obstacle which is placed at
             4.92 cm(I tried to keep it 5 cm away) from sensor at 0.0 degrees
             and is at 18.37 cm away from sensor at 74.64 degrees(which is maximum),
             For the foot sensor, the sensor shows a reading of about 10.70cm for maximum angle
             of 61 degrees and value of 15.10 for an angle of 0 degrees.
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

    def differential_control_callback(self, data):
        ''' Accepts incoming differential control values and simply relays them to the Autobed.
            This mode is used when Henry wants to control the autobed manually even if no sensors are present'''
        autobed_config_data = load_pickle(self.autobed_config_file)
        if data.data in CMDS:
            self.autobed_sender.write(autobed_config_data[data.data])
        else:
            self.autobed_u = np.asarray(autobed_config_data[data.data])
            u_thresh = np.array([80.0, 30.0, 50.0])
            l_thresh = np.array([1.0, 9.0, 1.0])
            self.autobed_u[self.autobed_u > u_thresh] = u_thresh[self.autobed_u > u_thresh]
            self.autobed_u[self.autobed_u < l_thresh] = l_thresh[self.autobed_u < l_thresh]
            #Make reached_destination boolean flase for all the actuators on the bed.
            self.reached_destination = False * np.ones(NUM_ACTUATORS)
            self.actuator_number = 0

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

    def update_autobed_configuration(self, req):
        ''' Is the callback from the GUI interaction Service. This service is called when the user wants to
        save a position to the Autobed. This function will update the autobed_config_data.yaml file with
        the input string and the present configuration of the bed. Once this is done,
        it will output success to the client'''

        try:
            #Get Autobed pickle file from the params directory into a dictionary
            current_autobed_config_data = load_pickle(self.autobed_config_file)
        except:
            return add_bed_configResponse(False)

        if req.config in CMDS:
            self.abdout1.publish(current_autobed_config_data.keys())
            return add_bed_configResponse(False)
        else:
            #Append present Autobed positions and angles to the dictionary
            current_autobed_config_data[req.config] = self.positions_in_autobed_units((self.prox_driver.get_sensor_data()[-1])[:NUM_ACTUATORS])
            try:
                #Update param file
                save_pickle(current_autobed_config_data, self.autobed_config_file)
                #Publish list of keys to the abdout1 topic
                self.abdout1.publish(current_autobed_config_data.keys())
                #Return success if param file correctly updated
                return add_bed_configResponse(True)
            except:
                return add_bed_configResponse(False)

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
                #If the error is greater than some allowed offset, then we actuate the motors to get closer to desired position'''
                if self.actuator_number < (NUM_ACTUATORS):
                    if abs(autobed_error[self.actuator_number]) > ERROR_OFFSET[self.actuator_number]:
                        self.autobed_sender.write(AUTOBED_COMMANDS[self.actuator_number][int(autobed_error[self.actuator_number]/abs(autobed_error[self.actuator_number]))])
                        self.reached_destination[self.actuator_number] = False
                    else:
                        self.reached_destination[self.actuator_number] = True
                        #We have reached destination for actuator self.actuator_number. Upgrade the actuation count
                        self.actuator_number += 1
            #If we have reached the destination position at all the actuators,
            #then publish the error and a boolean that says we have reached
            if self.reached_destination.all() == True:
                self.abdstatus0.publish(True)
            rate.sleep()

'''Runs the Autobed robot using an object of the AutobedClient class and the run method provided therein'''
if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="ROS Interface for AutoBed")
    parser.add_argument("serial_device", type=str, help="The serial device for the AutoBed Arduino serial connection.")
    parser.add_argument("baudrate", type=int, help="AutoBed Serial Baudrate")
    parser.add_argument("sensor_param_file", type=str, help="The paramter file describing the autobed sensors")
    parser.add_argument("number_of_sensors", type=int, help="Number of sensors on the AutoBed", default=4)
    parser.add_argument("autobed_config_file", type=sttr, help="Configuration file fo the AutoBed")
    args = parser.parse_args()
    #Initialize autobed node
    rospy.init_node('autobed_engine', anonymous = True)
    autobed = AutobedClient(args.serial_device,
                            args.autobed_config_file,
                            args.sensor_param_file,
                            args.baudrate,
                            args.num_of_sensors)
    autobed.run()
