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


class AccelerometerDriver(object):
    ''' A class for collecting raw data from a ADXL 335 accelerometer sensor
    attached to an arduino analog input device.
    '''
    def __init__(self, num_sensors, dev='/dev/ttyUSB0', baudrate=9600):
        #Accelerometer Bias Values 
        #TODO: Add API at a later stage to vary biases online.
        self.BIAS_Y = 331;
        self.BIAS_Z = 331;
        self.BIAS = np.array([self.BIAS_Y, self.BIAS_Z])
        self.num_sensors = num_sensors	
	self.num_analog = self.num_sensors*len(self.BIAS)
        self.serial_driver = serial_driver.SerialDriver(
                self.num_analog, dev, baudrate)
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
	    raw_acc_data = self._read_raw_data()
	    raw_acc_data = raw_acc_data[:4]
            raw_angle_data = self.acceleration_to_inclination(raw_acc_data)
        except ValueError as e:
	    print "Error in converting analog values to angles"
	    print e
	    sys.exit()
            raw_angle_data = np.zeros(self.num_sensors)
        raw_angle_data[np.where(raw_angle_data<0)[0]] = 0.
	return raw_angle_data

    def filter_data(self):
        '''Creates a low pass filter to filter out high frequency noise'''
        lpf = remez(self.bin_numbers, [0, 0.1, 0.25, 0.5], [1.0, 0.0])
        filt_data = np.zeros(self.num_sensors)
        for i in range(self.num_sensors):
            filt_data[i] = np.dot(lpf, self.collated_cal_data[:,i])
        return filt_data


    def acceleration_to_inclination(self, raw_data):
        '''Converts accelerometer output to inclination values'''
        angle_value = np.zeros(self.num_sensors)
        for i in range(self.num_sensors):
	    unbiased_raw = (raw_data[i*len(self.BIAS):(i+1)*len(self.BIAS)] - self.BIAS)
            angle_value[i] = (
                math.atan2(math.sqrt(unbiased_raw[0]**2), unbiased_raw[1] ) * (180.0/math.pi)) 
        return angle_value


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
    #pub0 = rospy.Publisher('/acc_cal_1', Float32)
    #pub1= rospy.Publisher('/acc_cal_2', Float32)
    pub2= rospy.Publisher('/acc_raw_1', Float32)
    #pub3= rospy.Publisher('/acc_raw_2', Float32)
    if len(sys.argv) < 3:
        sys.stderr.write('Usage: rosrun packagename adxl_accel_driver.py /dev/USBx')
        sys.exit(1)

    accel_driver = AccelerometerDriver(int(sys.argv[2]), dev=sys.argv[1], baudrate = 9600)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        data = accel_driver.get_sensor_data()
        raw_data = data
        #pub0.publish(Float32(cal_data[0]))
        #pub1.publish(Float32(cal_data[1]))
        pub2.publish(Float32(raw_data[0]))
