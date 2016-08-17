#!/usr/bin/env python

import sys
import serial
import numpy as np
import math
from scipy.signal import remez
from scipy.signal import lfilter

import roslib; roslib.load_manifest('autobed_engine')
import rospy, rosparam
from std_msgs.msg import Empty
from geometry_msgs.msg import Transform, Vector3, Quaternion
import serial_driver


class AutobedSensorDriver(object):
    ''' A class for collecting raw data from a ADXL 335 accelerometer sensor
    attached to an arduino analog input device.
    '''
    def __init__(self, num_sensors, dev='/dev/ttyUSB0', baudrate=9600):
        #Accelerometer Bias Values 
        #TODO: Add API at a later stage to vary biases online.
        self.BIAS_Y = 337;
        self.BIAS_Z = 337;
        self.BIAS = np.array([self.BIAS_Y, self.BIAS_Z])
        self.num_sensors = num_sensors	
        self.num_sharp_sensors = 1
        self.num_analog = self.num_sensors*len(self.BIAS) + self.num_sharp_sensors
        #Initialize arrays for holding sensor data
        self.cal_coefficients = 10.34297387 * np.ones(self.num_sharp_sensors) #Computed from calibration experiments. Will output in centimeters
        self.cal_offsets = -0.20397751 * np.ones(self.num_sharp_sensors) #Computed from calibration experiments. Will output in centimeters
        self.cal_exponents = -0.94007202 * np.ones(self.num_sharp_sensors) #Computed from calibration experiments. Will output in centimeters

        #Optional filtering initializations
        self.current_bin_number = 0
        self.bin_numbers = 7 
        self.raw_dat_array = np.zeros((self.bin_numbers, self.num_sensors+self.num_sharp_sensors))
        self.filtered_data = np.zeros(self.num_sensors + self.num_sharp_sensors)

        self.serial_driver = serial_driver.SerialDriver(self.num_analog, dev, baudrate)
        good_data = False
        test_rate = rospy.Rate(1)
        while not good_data and not rospy.is_shutdown():
            try:
                self._read_raw_data()
                good_data = True
            except ValueError as e:
                print "No real data from sensor, cannot get inital zero"
                test_rate.sleep()
                pass
        print "[accel_sensor_driver]: Finished initialization"


    def _read_raw_data(self):
        try:
            raw_data = np.array(self.serial_driver.getValues())
            if any(np.isnan(raw_data)):
		print raw_data
                raise ValueError("Raw data from accelerometer contains NaN")
        except ValueError as e:
            print "[accel_sensor_driver]: Warning: %s" %e
            raise e
        except:
          print "Reading data in accelerometer driver failed"
          raise
        assert len(raw_data) == self.num_analog, "Size of incoming data from sensor doesnt match # of sensors."
        return raw_data


    def get_sensor_data(self):
        try:
            dat = self._read_raw_data()
            raw_acc_data = dat[:4]
            raw_sharp_data = dat[4:]
            raw_angle_data = self.acceleration_to_inclination(raw_acc_data)
	    raw_dist_data = self.calibrate_data(raw_sharp_data)
        except ValueError as e:
            print "Error in converting analog values to angles"
            print e
            sys.exit()
            raw_angle_data = np.zeros(self.num_sensors)
	    raw_dist_data = np.zeros(self.num_sharp_sensors)
        raw_angle_data[np.where(raw_angle_data<0)[0]] = 0.
        raw_dist_data[np.where(raw_angle_data<0)[0]] = 0.
        raw_data = np.array([raw_angle_data[0], raw_dist_data[0], raw_angle_data[1]])
        #Optional filtering.
        if self.current_bin_number >= (self.bin_numbers):
            self.raw_dat_array = np.delete(self.raw_dat_array, 0, 0)
            self.raw_dat_array = np.append(self.raw_dat_array, [raw_data], axis = 0)
            filtered_data = self.filter_data()
        else:
            self.raw_dat_array[self.current_bin_number,:] += raw_data
            filtered_data = raw_data
            self.current_bin_number += 1
        return filtered_data

    def filter_data(self):
        '''Creates a low pass filter to filter out high frequency noise'''
        lpf = remez(self.bin_numbers, [0, 0.1, 0.25, 0.5], [1.0, 0.0])
        filt_data = np.zeros(self.num_sensors+self.num_sharp_sensors)
        for i in range(self.num_sensors+self.num_sharp_sensors):
            filt_data[i] = np.dot(lpf, self.raw_dat_array[:,i])
        return filt_data


    def acceleration_to_inclination(self, raw_data):
        '''Converts accelerometer output to inclination values'''
        angle_value = np.zeros(self.num_sensors)
        for i in range(self.num_sensors):
	    unbiased_raw = (raw_data[i*len(self.BIAS):(i+1)*len(self.BIAS)] - self.BIAS)
            angle_value[i] = (
                math.atan2(math.sqrt(unbiased_raw[0]**2), unbiased_raw[1] ) * (180.0/math.pi)) 
        return angle_value

    def calibrate_data(self, raw_data):
        data = raw_data[raw_data<=0.]=1.0E-8
       #return self.cal_coefficients * raw_data ** self.cal_exponents
        return self.cal_coefficients * (self.to_voltage(raw_data) + self.cal_offsets) ** self.cal_exponents

    def to_voltage(self, raw_data):
        '''Converts ADC values to voltage readings. Assumes 10 bit ADC, and 5V 
        ADC reference voltage for Vcc = 5Volts'''
        return 5.0*raw_data/1024.0



    def modify_accelerometer_bias(self, new_bias):
        '''CAUTION: USE THIS FUNCTION WITH CARE. IT CAN THROW THE ACCELEROMETER
        CALIBRATION OFF.
        Inputs: List of 3 bias values for eg. [370.0, 331.0, 339.0]
        Output: Boolean to confirm/deny change of accelerometer bias.'''
        try:
            self.BIAS = new_bias[:]
            return True
        except: 
            return False


if __name__=='__main__':
    rospy.init_node('accelerometer_sensor_driver')
    from std_msgs.msg import Float32
    #import pickle as pkl
    import time 
    #import matplotlib.pyplot as plt

    #plt.axis([0, 1000, 0, 90])
    #plt.ion()
    try:
        mean_array = pkl.load(open('/home/mycroft/mean_array2.pkl', 'rb'))
    except:
	    mean_array = []
    try:
        std_array =  pkl.load(open('/home/mycroft/std_array2.pkl', 'rb'))
    except:
        std_array = []
   
    if len(sys.argv) < 3:
        sys.stderr.write('Usage: rosrun packagename adxl_accel_driver.py /dev/USBx')
        sys.exit(1)

    accel_driver = AutobedSensorDriver(int(sys.argv[2]), dev=sys.argv[1], baudrate = 9600)

    dat_array = np.array([0])
    rate = rospy.Rate(10)
    i=0
    x = [0]
    t_end = time.time() + 10
    while time.time() < t_end:
        data = accel_driver.get_sensor_data()
        raw_data = data
        print i, raw_data
        i+=1
    sys.exit()
