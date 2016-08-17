#!/usr/bin/env python
import sys
import serial
import time
import numpy as np
import math
import threading 

import roslib; roslib.load_manifest('autobed_engine')
import rospy
import serial_driver
#import sharp_prox_driver
import adxl_accel_driver
import autobed_adxl_sharp_driver
import cPickle as pkl
from hrl_lib.util import save_pickle, load_pickle

import websocket
import atexit

from std_msgs.msg import Bool, Float32, String, Int16
from scipy.signal import remez
from scipy.signal import lfilter
from hrl_msgs.msg import FloatArrayBare, StringArray
from geometry_msgs.msg import TransformStamped 
from geometry_msgs.msg import Transform, Vector3, Quaternion
from autobed_engine.srv import *

#This is the maximum error allowed in our control system.
ERROR_OFFSET = [2, 1, 4] #[degrees, centimeters , degrees]
"""Number of Actuators"""
NUM_ACTUATORS = 3
""" Basic Differential commands to the Autobed via GUI"""
CMDS = ['headUP', 
        'headDN', 
        'bedUP', 
        'bedDN', 
        'legsUP', 
        'legsDN']
"""List of positive movements"""
AUTOBED_COMMANDS = [[0, 'headUP', 'headDN'], 
                    [0, 'bedUP', 'bedDN'], 
                    [0, 'legsUP', 'legsDN']]
timers = {}

##
#Class AutobedClient()
#gives interface to connect to the Autobed, that contains methods to run the 
#autobed engine, and access the sharp IR sensors on board the bed.
#
class AutobedClient():
    """Gives interface to connect to the Autobed, that contains methods to run
    the autobed engine, and access the sharp IR sensors on board the bed."""
    def __init__(self, dev, autobed_config_file, param_file, 
            baudrate, num_of_sensors, sensor_type):
        '''Autobed engine node that listens into the base selection output data 
        array and feeds the same as an input to the autobed control system. 
        Further, it listens to the sensor position'''
        self.SENSOR_TYPE = sensor_type
        #self.u_thresh = np.array([70.0, 41.0, 45.0])
        self.u_thresh = np.array([70.0, 20.0, 45.0])
        #self.l_thresh = np.array([0.0, 9.0, 1.0])
        self.l_thresh = np.array([0.0, 0.0, 1.0])
        self.dev = dev
        self.autobed_config_file = autobed_config_file
        self.param_file = param_file
        self.baudrate = baudrate
        self.num_of_sensors = num_of_sensors
        self.reached_destination = True * np.ones(NUM_ACTUATORS)
        self.actuator_number = 0
	self.frame_lock = threading.RLock()
	self.head_filt_data = 0
	self.bin_numbers = 11
	self.collated_head_angle = np.ones((self.bin_numbers, 1))
	self.lpf = remez(self.bin_numbers, [0, 0.05, 0.1, 0.5], [1.0, 0.0])
        #Create a proximity sensor object
        #self.prox_driver = (
        #        sharp_prox_driver.ProximitySensorDriver(
        #            int(num_of_sensors), 
        #            param_file = self.param_file,
        #            dev = self.dev,
        #            baudrate = self.baudrate))

        if self.SENSOR_TYPE == 'MOCAP':
            self.leg_angle = 0
            self.head_angle = 0
            rospy.Subscriber("/abd_head_angle/pose", TransformStamped, 
                                                self.autobed_head_angle_cb)
            rospy.Subscriber("/abd_leg_angle/pose", TransformStamped, 
                                                self.autobed_leg_angle_cb)
	elif self.SENSOR_TYPE == 'ADXL_SHARP':
	    self.acc_driver = (
            autobed_adxl_sharp_driver.AutobedSensorDriver(
            	    2,
                    dev = self.dev,
                    baudrate = self.baudrate))
	elif self.SENSOR_TYPE == 'ADXL_KINECT':
	    self.bed_ht = 0
	    self.acc_driver = (adxl_accel_driver.AccelerometerDriver(2,
                    					dev = self.dev,
                    					baudrate = self.baudrate))
            rospy.Subscriber("/bed_ht", Float32, self.kinect_bed_ht_cb)

    
        #Let the sensors warm up
        rospy.sleep(3.)
        # Input to the control system.
        self.autobed_u = (self.get_sensor_data())
        #Start a publisher to publish autobed status and error
        self.abdout0 = rospy.Publisher("/abdout0", FloatArrayBare)
        self.abdstatus0 = rospy.Publisher("/abdstatus0", Bool)
        self.abdout1 = rospy.Publisher("/abdout1", StringArray, latch=True)
        #Start a subscriber to run the autobed engine when we get a command
        rospy.Subscriber("/abdin0", FloatArrayBare, 
                self.autobed_engine_callback)
        #Start a subscriber that takes a differential input & relays it to bed
        rospy.Subscriber("/abdin1", String, self.differential_control_callback)
        #Set up the service that stores present Autobed configuration
        rospy.Service('add_autobed_config', update_bed_config, 
                self.add_autobed_configuration)
        #Set up the service that deletes specified Autobed configuration  
        rospy.Service('delete_autobed_config', update_bed_config, 
                self.delete_autobed_configuration)
        try:
            init_autobed_config_data = load_pickle(self.autobed_config_file)
            self.abdout1.publish(init_autobed_config_data.keys())
        except:
            init_autobed_config_data = {}
            save_pickle(init_autobed_config_data, self.autobed_config_file)
            self.abdout1.publish(init_autobed_config_data.keys())
	self.ws = websocket.create_connection("ws://localhost:828")
        print '*** Autobed 2.0 Ready ***'

    def filter_head_data(self):
        '''Creates a low pass filter to filter out high frequency noise'''
        if np.shape(self.lpf) == np.shape(self.collated_head_angle):
            self.head_filt_data = np.dot(self.lpf, self.collated_head_angle)
	    #self.head_filt_data = self.collated_head_angle[-1]
        else:
            pass
        return

    def websocket_cleanup(self):
	self.ws.close()

    def diff_motion(self, message):
        '''will send differential command message to websocket'''
	try:
	    self.ws.send(message)
	except:
	    return


    def positions_in_autobed_units(self, distances):
        ''' Accepts position of the obstacle which is placed at 
        4.92 cm(I tried to keep it 5 cm away) from sensor at 0.0 degrees 
        and is at 18.37 cm away from sensor at 74.64 degrees(which is maximum),
        For the foot sensor, the sensor shows a reading of about 10.70cm for 
        maximum angle of 61 degrees and value of 15.10 for an angle of 0 degrees
        and returns value of the head tilt in degrees'''
        if distances[0] <= 8.87:
            distances[0] = (2.7615*distances[0] - 14.0838)
        elif distances[0] > 8.87 and distances[0] <= 11.2:
            distances[0] = (3.5416*distances[0] - 21.0032)
        elif distances[0] > 11.2 and distances[0] <= 15.5:
            distances[0] = (3.5337*distances[0] - 20.9147)
        elif distances[0] > 15.5 and distances[0] <= 20.5:
            distances[0] = (1.96*distances[0] + 3.53008)
        elif distances[0] > 20.5 and distances[0] <= 23.5:
            distances[0] = (3.7827*distances[0] - 33.9773)
        elif distances[0] > 23.5 and distances[0] <= 26:
            distances[0] = (10*distances[0] - 180)
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


    def autobed_head_angle_cb(self, data):
        '''These angles are the ground truth obtained from the markers placed
        on the autobed'''
        q0 = data.transform.rotation.x
        q1 = data.transform.rotation.y
        q2 = data.transform.rotation.z
        q3 = data.transform.rotation.w
        self.head_angle = 180 + math.atan2(2*(q0*q3 + q1*q2), (1 - 
                                    2*(q2**2 + q3**2)))*(180.0/ math.pi) 

    def autobed_leg_angle_cb(self, data):
        '''These angles are the ground truth obtained from the markers placed
        on the autobed'''
        q0 = data.transform.rotation.x
        q1 = data.transform.rotation.y
        q2 = data.transform.rotation.z
        q3 = data.transform.rotation.w
        self.leg_angle = 180 - math.atan2(2*(q0*q3 + q1*q2), (1 - 
                                    2*(q2**2 + q3**2)))*(180.0/ math.pi) 

    def kinect_bed_ht_cb(self, data):
        '''Gets bed height from kinect publisher on Mac Mini'''
        self.bed_ht = data.data

    def differential_control_callback(self, data):
        ''' Accepts incoming differential control values and simply relays them 
        to the Autobed. This mode is used when Henry wants to control the 
        autobed manually even if no sensors are present'''

	print "Received Differential Control Data"
	print data.data
        autobed_config_data = load_pickle(self.autobed_config_file) 
	with self.frame_lock:
		if data.data in CMDS: 
		    self.diff_motion(data.data)
		else:
		    self.autobed_u = np.asarray(autobed_config_data[data.data])
		    self.autobed_u[self.autobed_u > self.u_thresh] = (
			    self.u_thresh[self.autobed_u > self.u_thresh])
		    self.autobed_u[self.autobed_u < self.l_thresh] = (
			    self.l_thresh[self.autobed_u < self.l_thresh])
		    #Make reached_destination boolean false for all the actuators 
		    self.reached_destination = False * np.ones(NUM_ACTUATORS)
		    self.actuator_number = 0


    def autobed_engine_callback(self, data):
        ''' Accepts incoming position values from the base selection algorithm 
        and assigns it to a global variable. This variable is then used to guide 
        the autobed to the desired position using the engine'''
	with self.frame_lock:
		current_autobed_pose = self.get_sensor_data()
		self.autobed_u = np.asarray(data.data)
		#We threshold the incoming data
		self.autobed_u[self.autobed_u > self.u_thresh] = (
			self.u_thresh[self.autobed_u > self.u_thresh])
		self.autobed_u[self.autobed_u < self.l_thresh] = (
			self.l_thresh[self.autobed_u < self.l_thresh])
		for i in range(len(self.autobed_u)):
		    if math.isnan(self.autobed_u[i]):
			self.autobed_u[i] = current_autobed_pose[i] 
		if math.isnan(self.autobed_u.all()):
		    self.autobed_kill()
		    return
		#Make reached_destination boolean flase for all the actuators on the bed
		self.reached_destination = False * np.ones(NUM_ACTUATORS)
		self.actuator_number = 0


    def delete_autobed_configuration(self, req):
        """ Is the callback from the GUI Interaction Service. This service is 
        called when a user wants to delete a saved position. This function will 
        update the autobed_config_data.yaml file by removing the data 
        corresponding to the string specified. If successful, it will return a 
        boolean value"""
        try:
            #Get Autobed pickle file from the params directory into a dictionary
            current_autobed_config_data = load_pickle(self.autobed_config_file)
        except: 
            #return zero
            return update_bed_configResponse(False)

        #Delete present Autobed positions and angles from the dictionary
        try:
            current_autobed_config_data.pop(req.config)
        except KeyError:
            update_bed_configResponse(False)
            print "Autobed Configuration Database is already empty"
        try:
            #Update param file
            save_pickle(current_autobed_config_data, self.autobed_config_file) 
            #Publish list of keys to the abdout1 topic
            self.abdout1.publish(current_autobed_config_data.keys())
            #Return success if param file correctly updated
            return update_bed_configResponse(True)
        except:
            return update_bed_configResponse(False)


    def add_autobed_configuration(self, req):
        ''' Is the callback from the GUI interaction Service. This service is 
        called when the user wants to save a position to the Autobed. This 
        function will update the autobed_config_data.yaml file with the input 
        string and the present configuration of the bed. Once this is done, it 
        will output success to the client'''
        try:
            #Get Autobed pickle file from the params directory into a dictionary
            current_autobed_config_data = load_pickle(self.autobed_config_file)
        except:
            #return zero
            return update_bed_configResponse(False)

        if req.config in CMDS:
            self.abdout1.publish(current_autobed_config_data.keys())
            return update_bed_configResponse(False)
        else:
            #Append present Autobed positions and angles to the dictionary
	    current_raw = self.get_sensor_data()
            with self.frame_lock:
            	self.collated_head_angle = np.delete(self.collated_head_angle, 0)
           	self.collated_head_angle = np.append(self.collated_head_angle, [current_raw[0]])
	    #Filter Head Angle
	    #self.filter_head_data()
	    #current_filtered = np.array([self.head_filt_data, current_raw[1], current_raw[2]])
	    current_filtered = np.array([current_raw[0], current_raw[1], current_raw[2]])
            current_autobed_config_data[req.config] = (current_filtered)
	    print "Saved:"
	    print current_filtered
            try:
                #Update param file
                save_pickle(current_autobed_config_data, 
                        self.autobed_config_file)
                #Publish list of keys to the abdout1 topic
                self.abdout1.publish(current_autobed_config_data.keys())
                #Return success if param file correctly updated
                return update_bed_configResponse(True)
            except:
                return add_bed_configResponse(False)


    def get_sensor_data(self):
        '''Returns data from the sensor selected by the variable self.SENSOR_TYPE.
        If Mocap is selected, it will return mocap angles, else, it will return
        angles from the SHARP IR Sensors'''
        if self.SENSOR_TYPE == 'MOCAP':
            bed_ht = (self.prox_driver.get_sensor_data()[-1])[1]
            return np.asarray([self.head_angle, bed_ht, self.leg_angle])
        elif self.SENSOR_TYPE == 'SHARP':
            return np.asarray(self.positions_in_autobed_units((
                        self.prox_driver.get_sensor_data()[-1])[:NUM_ACTUATORS]))
	elif self.SENSOR_TYPE == 'ADXL_SHARP':
	    total_dat = self.acc_driver.get_sensor_data()
	    #bed_ht = 0#(self.prox_driver.get_sensor_data()[-1])[-1]
	    #bed_angles = self.acc_driver.get_sensor_data()
	    #return np.asarray([bed_angles[0], bed_ht, bed_angles[1]])
	    return np.asarray(total_dat)
        elif self.SENSOR_TYPE == 'ADXL_KINECT':
	    bed_ht = self.bed_ht
	    bed_angles = self.acc_driver.get_sensor_data()
	    return np.asarray([bed_angles[0], bed_ht, bed_angles[1]])

    def autobed_kill(self):
        '''Kills the autobed, while its going to a predefined location'''
        self.autobed_u = self.get_sensor_data() 
        self.reached_destination = True*np.ones(len(self.reached_destination))

    def run(self):
        rate = rospy.Rate(10) #5 Hz
        #Variable that denotes what actuator is presently being controlled
        self.actuator_number = 0 
        '''Initialize the autobed input to the current sensor values, 
        so that the autobed doesn't move unless commanded'''
        self.autobed_u =  self.get_sensor_data() 
	self.collated_head_angle = self.collated_head_angle*self.autobed_u[0]
        while not rospy.is_shutdown():
	    #Publish present Autobed sensor readings
	    current_raw = self.get_sensor_data()
            self.abdout0.publish(current_raw)
	    with self.frame_lock:
            	self.collated_head_angle = np.delete(self.collated_head_angle, 0)
           	self.collated_head_angle = np.append(self.collated_head_angle, [current_raw[0]])
            if self.reached_destination.all() == False:
                '''If the error is greater than some allowed offset, 
                then we actuate the motors to get closer to desired position'''
		#Filter Head Angle
		#self.filter_head_data()
		#current_filtered = np.array([self.head_filt_data, current_raw[1], current_raw[2]])
		current_filtered = np.array([current_raw[0], current_raw[1], current_raw[2]])
            	autobed_error = np.asarray(self.autobed_u - current_filtered) 
		#TODO: Remove the line below when using legs.
		autobed_error[2] = 0.0
                if self.actuator_number < (NUM_ACTUATORS):
                    if abs(autobed_error[self.actuator_number]) > (
                            ERROR_OFFSET[self.actuator_number]):
                        self.reached_destination[self.actuator_number] = False
                        self.diff_motion(
                                AUTOBED_COMMANDS[self.actuator_number][int(
                                    autobed_error[self.actuator_number]/abs(
                                        autobed_error[self.actuator_number]))])
		        print current_filtered
                    else:
                        self.reached_destination[self.actuator_number] = True
                        #We have reached destination for current actuator 
                        #Upgrade the actuation count
                        self.actuator_number += 1
            #If we have reached the destination position at all the actuators,
            #then publish the error and a boolean that says we have reached
            if self.reached_destination.all() == True:
                self.actuator_number = 0
                self.abdstatus0.publish(True)
            else:
                self.abdstatus0.publish(False)
            rate.sleep()


if __name__ == "__main__":
    '''Runs the Autobed robot using an object of the 
    AutobedClient class and the run method provided therein'''
    import argparse
    parser = argparse.ArgumentParser(description="ROS Interface for AutoBed")
    parser.add_argument("serial_device", type=str, 
            help="The serial device for the AutoBed Arduino serial connection.")
    parser.add_argument("baudrate", 
            type=int, help="AutoBed Serial Baudrate")
    parser.add_argument("sensor_param_file", 
            type=str, help="The paramter file describing the autobed sensors")
    parser.add_argument("number_of_sensors", 
            type=int, help="Number of sensors on the AutoBed", default=4)
    parser.add_argument("autobed_config_file", 
            type=str, help="Configuration file fo the AutoBed")
    parser.add_argument("sensor_type", 
            type=str, help="What sensor are you using for the Autobed: MOCAP vs. SHARP vs. ADXL_SHARP vs ADXL_KINECT"
	    , default="ADXL_KINECT")

    args = parser.parse_args(rospy.myargv()[1:])
    #Initialize autobed node
    rospy.init_node('autobed_engine', anonymous = True)
    autobed = AutobedClient(args.serial_device,
                            args.autobed_config_file,
                            args.sensor_param_file,
                            args.baudrate,
                            args.number_of_sensors,
			    args.sensor_type)

    atexit.register(autobed.websocket_cleanup)
    autobed.run()
