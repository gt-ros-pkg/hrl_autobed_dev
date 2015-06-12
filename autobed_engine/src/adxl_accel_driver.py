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
    def __init__(self, num_sensors, param_file=None, 
                                            dev='/dev/ttyUSB0', baudrate=9600):
        #Accelerometer Bias Values 
        #TODO: Add API at a later stage to vary biases online.
        self.BIAS_X = 370;
        self.BIAS_Y = 330;
        self.BIAS_Z = 272;
        self.BIAS = [self.BIAS_X, self.BIAS_Y, self.BIAS_Z]
        self.num_sensors = num_sensors
        self.serial_driver = serial_driver.SerialDriver(
                self.num_sensors*len(self.BIAS), dev, baudrate)
        self.zero_sensor_sub = rospy.Subscriber('/autobed_engine/accelerometer', 
                Empty, self.zero_sensor_cb)
        #Initialize arrays for holding sensor data
        self.cal_coefficients = 1.0 * np.ones(self.num_sensors) 
        self.cal_offsets = 0.0 * np.ones(self.num_sensors) #
        self.cal_exponents = 0.0 * np.ones(self.num_sensors) #
        self.frames = np.zeros(self.num_sensors)
        self.sensor_positions = np.zeros( [3, self.num_sensors] )
        self.sensor_quats =  np.zeros( [4, self.num_sensors] )
        #Optional filtering initializations
        self.current_bin_number = 0
        self.bin_numbers = 21 
        self.collated_cal_data = np.zeros((self.bin_numbers,
            self.num_sensors))
        self.filtered_data = np.zeros(self.num_sensors)

        if param_file is None:
            rospy.logwarn(
                    '[accel_sensor_driver] No parameters loaded for Sensors')
        else:
            sensor_config_data = rosparam.load_file(param_file)[0][0]
            for channel, sensor in sensor_config_data.iteritems():
                self.frames[channel] = sensor['normal_pose']['frame']
                self.cal_coefficients[channel] = (
                                        sensor['calibration']['coefficient'])
                self.cal_offsets[channel] = sensor['calibration']['offset']
                self.cal_exponents[channel] = sensor['calibration']['exponent']
                self.sensor_positions[:,channel] =  (
                                            sensor['normal_pose']['position'])
                self.sensor_quats[:,channel] =  (
                                        sensor['normal_pose']['orientation'])
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
        assert len(raw_data) == self.num_sensors*len(self.BIAS), "Size of incoming data from sensor doesnt match # of sensors."
        return raw_data


    def get_sensor_data(self):
        try:
            raw_data = self.acceleration_to_inclination(
                    self._read_raw_data()[:self.num_sensors*len(self.BIAS)])
        except ValueError as e:
            raw_data = np.zeros(self.num_sensors)
        cal_data = self.calibrate_data(raw_data)
        cal_data[np.where(cal_data<0)[0]] = 0.
        #Optional filtering.
        if self.current_bin_number >= (self.bin_numbers-1):
            self.collated_cal_data = np.delete(self.collated_cal_data, 0, 0)
            self.collated_cal_data = np.append(self.collated_cal_data, 
                                                        [cal_data], axis = 0)
            filtered_data = self.filter_data()
        else:
            self.collated_cal_data[self.current_bin_number, :] += cal_data
            filtered_data = cal_data
            self.current_bin_number += 1
        return (self.frames, self.sensor_positions, 
                        self.sensor_quats, raw_data, cal_data, filtered_data)


    def calibrate_data(self, raw_data):
        data = raw_data[raw_data<=0.]=1.0E-8
       #return self.cal_coefficients * raw_data ** self.cal_exponents
        return (self.cal_coefficients * (raw_data 
                                    + self.cal_offsets) ** self.cal_exponents)


    def filter_data(self):
        '''Creates a low pass filter to filter out high frequency noise'''
        lpf = remez(self.bin_numbers, [0, 0.1, 0.25, 0.5], [1.0, 0.0])
        filt_data = np.zeros(self.num_sensors)
        for i in range(self.num_sensors):
            filt_data[i] = np.dot(lpf, self.collated_cal_data[:,i])
        return filt_data


    def link_name_cb(self, req):
        return None_StringResponse(data='/torso_lift_link')


    def acceleration_to_inclination(self, raw_data):
        '''Converts accelerometer output to inclination values'''
        unbiased_raw = (raw_data - np.asarray(self.BIAS*self.num_sensors))
        angle_value = np.zeros(self.num_sensors)
        for i in range(self.num_sensors):
            angle_value[i] = (
                math.atan2( math.sqrt(unbiased_raw[i*len(self.BIAS) + 1]**2), 
                unbiased_raw[i*len(self.BIAS) + 2] ) * (180.0/math.pi)) 
        return angle_value

    def zero_sensor_cb(self, msg):
        raw_data = self._read_raw_data()
        assert len(raw_data) == len(self.intercepts), "Cannot re-zero angle sensor, size of current raw data does not match list of intercepts"
        self.intercepts = np.array(raw_data)


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
    pub0 = rospy.Publisher('/acc_cal_1', Float32)
    pub1= rospy.Publisher('/acc_cal_2', Float32)
    pub2= rospy.Publisher('/acc_raw_1', Float32)
    pub3= rospy.Publisher('/acc_raw_2', Float32)
    if len(sys.argv) < 3:
        sys.stderr.write('Usage: rosrun packagename adxl_accel_driver.py /dev/USBx number_of_sensors param_file_path')
        sys.exit(1)

    accel_driver = AccelerometerDriver(int(sys.argv[2]), sys.argv[3], 
                                            dev=sys.argv[1], baudrate = 9600)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        data = accel_driver.get_sensor_data()
        raw_data = data[-3]
        cal_data = data[-1]

        pub0.publish(Float32(cal_data[0]))
        pub1.publish(Float32(cal_data[1]))
 
        pub2.publish(Float32(raw_data[0]))
        pub3.publish(Float32(raw_data[1]))
        rate.sleep()
