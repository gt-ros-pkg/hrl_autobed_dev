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

from std_msgs.msg import Bool, Float32, String, Int16
from scipy.signal import remez
from scipy.signal import lfilter
from hrl_msgs.msg import FloatArrayBare, StringArray
from geometry_msgs.msg import TransformStamped 
from geometry_msgs.msg import Transform, Vector3, Quaternion

#This is the maximum error allowed in our control system.
ERROR_OFFSET = [2., 0.5, 4.] #[degrees, centimeters , degrees]
MINIMUM_PROGRESS = [2., 1., 4.]
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
    def __init__(self, dev, autobed_config_file, param_file, baudrate, num_of_sensors, sensor_type):
        '''Autobed engine node that listens into the base selection output data 
        array and feeds the same as an input to the autobed control system. 
        Further, it listens to the sensor position'''
        self.SENSOR_TYPE = sensor_type
        self.this_actuator_movement_timer = rospy.Time.now()
        self.progress_stalled = False
        self.progress_counter = 0
        #self.u_thresh = np.array([70.0, 41.0, 45.0])
        #self.u_thresh = np.array([65.0, 8.0, 45.0])
        self.u_thresh = np.array([70.0, 20.0, 45.0])
        #self.l_thresh = np.array([0.0, 9.0, 1.0])
        self.l_thresh = np.array([0.0, 0.0, 1.0])
        self.dev = dev
        self.autobed_config_file = autobed_config_file
        try:
            self.autobed_config_data = load_pickle(self.autobed_config_file)
        except:
            self.autobed_config_data = {}
            save_pickle(self.autobed_config_data, self.autobed_config_file)

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
            self.acc_driver = (autobed_adxl_sharp_driver.AutobedSensorDriver(2, dev = self.dev, baudrate = self.baudrate))
        elif self.SENSOR_TYPE == 'ADXL_and_HOKUYO':
            self.bed_ht = 0
            self.acc_driver = (adxl_accel_driver.AccelerometerDriver(2, dev=self.dev, baudrate=self.baudrate))
            rospy.Subscriber("/bed_ht", Float32, self.hokuyo_bed_ht_cb)

        #Let the sensors warm up
        rospy.sleep(3.)
        # Input to the control system.
        self.autobed_u = (self.get_sensor_data())
        #Start a publisher to publish autobed status and error
        self.abdout0 = rospy.Publisher("/abdout0", FloatArrayBare)
        self.abdstatus0 = rospy.Publisher("/abdstatus0", Bool)

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
        with self.frame_lock:
            q0 = data.transform.rotation.x
            q1 = data.transform.rotation.y
            q2 = data.transform.rotation.z
            q3 = data.transform.rotation.w
            self.head_angle = 180 + math.atan2(2*(q0*q3 + q1*q2), (1 - 2*(q2**2 + q3**2)))*(180.0/ math.pi)

    def autobed_leg_angle_cb(self, data):
        '''These angles are the ground truth obtained from the markers placed
        on the autobed'''
        with self.frame_lock:
            q0 = data.transform.rotation.x
            q1 = data.transform.rotation.y
            q2 = data.transform.rotation.z
            q3 = data.transform.rotation.w
            self.leg_angle = 180 - math.atan2(2*(q0*q3 + q1*q2), (1 -
                                                                  2*(q2**2 + q3**2)))*(180.0/ math.pi)

    def hokuyo_bed_ht_cb(self, data):
        '''Gets bed height from hokuyo publisher on Mac Mini'''
        with self.frame_lock:
            self.bed_ht = data.data


    def get_sensor_data(self):
        '''Returns data from the sensor selected by the variable self.SENSOR_TYPE.
        If Mocap is selected, it will return mocap angles, else, it will return
        angles from the SHARP IR Sensors'''
        if self.SENSOR_TYPE == 'MOCAP':
            bed_ht = (self.prox_driver.get_sensor_data()[-1])[1]
            return np.asarray([self.head_angle, bed_ht, self.leg_angle])
        elif self.SENSOR_TYPE == 'SHARP':
            return np.asarray(self.positions_in_autobed_units((self.prox_driver.get_sensor_data()[-1])[:NUM_ACTUATORS]))
        elif self.SENSOR_TYPE == 'ADXL_SHARP':
            total_dat = self.acc_driver.get_sensor_data()
            #bed_ht = 0#(self.prox_driver.get_sensor_data()[-1])[-1]
            #bed_angles = self.acc_driver.get_sensor_data()
            #return np.asarray([bed_angles[0], bed_ht, bed_angles[1]])
            return np.asarray(total_dat)
        elif self.SENSOR_TYPE == 'ADXL_and_HOKUYO':
            bed_ht = self.bed_ht
            bed_angles = self.acc_driver.get_sensor_data()
            bed_angles[0] -= 20
            return np.asarray([bed_angles[0], bed_ht, bed_angles[1]])


    def run(self):
        rate = rospy.Rate(10) #5 Hz
        #Variable that denotes what actuator is presently being controlled
        self.actuator_number = 0 
        '''Initialize the autobed input to the current sensor values, 
        so that the autobed doesn't move unless commanded'''
        self.autobed_u = self.get_sensor_data()
        self.collated_head_angle = self.collated_head_angle*self.autobed_u[0]
        self.progress_counter = 0
        self.progress_stalled = False
        while not rospy.is_shutdown():
            #Publish present Autobed sensor readings
            #print 'Current actuator', self.actuator_number
            current_raw = self.get_sensor_data()
            #print 'Current state:', current_raw
            self.abdout0.publish(current_raw)



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
                        type=str, help="What sensor are you using for the Autobed: MOCAP vs. SHARP vs. ADXL_SHARP vs ADXL_and_HOKUYO"
                        , default="ADXL_and_HOKUYO")

    args = parser.parse_args(rospy.myargv()[1:])
    #Initialize autobed node
    rospy.init_node('autobed_engine')
    autobed = AutobedClient(args.serial_device,
                            args.autobed_config_file,
                            args.sensor_param_file,
                            args.baudrate,
                            args.number_of_sensors,
                            args.sensor_type)
    autobed.run()
