#!/usr/bin/env python

import sys
import serial
import numpy as np

import roslib; roslib.load_manifest('hrl_autobed_dev')
import rospy, rosparam
from std_msgs.msg import Empty
from geometry_msgs.msg import Transform, Vector3, Quaternion

from m3skin_ros.srv import None_TransformArray, None_TransformArrayResponse
from m3skin_ros.srv import None_String, None_StringResponse

import serial_driver
import sharp_prox_driver

class SensorCalibrationEngine(object):
    '''A class that helps generate data for the SHARP sensors 
    that make calibration easier'''
    def __init__(self, num_sensors, param_file=None, dev='/dev/ttyUSB0', baudrate=115200):
        self.num_sensors = num_sensors
        self.prox_driver = sharp_prox_driver.ProximitySensorDriver(self.num_sensors, param_file, dev, baudrate)

    def get_calibration_data(self):
        '''This function reads in serially, values from the FIRST SHARP SENSOR, and 
        packs them up into an array of two values: RAW_DATA_FROM_ADC, CONVERTED_VOLTAGE'''
        raw_data_all_sensors = self.prox_driver.get_sensor_data()[-2]
        raw_data_first_sensor = raw_data_all_sensors[0]
        return raw_data_first_sensor, self.to_voltage(raw_data_first_sensor)
        
    def to_voltage(self, raw_data):
        '''Converts ADC values to voltage readings. Assumes 10 bit ADC, and 5V 
        ADC reference voltage for Vcc = 5Volts'''
        return 5.0*raw_data/1024.0





if __name__=='__main__':
    rospy.init_node('prox_cal_engine')
    from std_msgs.msg import Float32
    '''Create publishers to publish all the important information computed by the 
    calibration engine'''
    pub0 = rospy.Publisher('/raw_data', Float32)
    pub1 = rospy.Publisher('/raw_voltages', Float32)
    pub2 = rospy.Publisher('/mean_voltage', Float32)
     
    if len(sys.argv) < 3:
        sys.stderr.write('Usage: rosrun packagename sharp_sensor_cal.py  /dev/USBx number_of_sensors param_file_path')
        sys.exit(1)
    
    sensor_cal = SensorCalibrationEngine(int(sys.argv[2]), sys.argv[3], dev=sys.argv[1])
    
    #Allowing human to initialize sensor
    print 'Initialize calibration apparatus to a certain distance from the sensor.\n Note down the distance.' 
    rospy.sleep(10.)
    print 'Beginning calibration...'

    num_samples = 0 
    raw_voltages = np.zeros(10000)
    mean_voltage = 0.0
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        data = sensor_cal.get_calibration_data()
        raw_data = data[0]
        raw_voltages[num_samples] = data[1]
        #Print sampled values in terms of ADC output and voltages
        pub2.publish(Float32(mean_voltage)) 
        pub0.publish(Float32(raw_data))
        pub1.publish(Float32(raw_voltages[num_samples]))
        #After 10 secs of holding the apparatus still, we compute the mean volage and publish it
        if num_samples >= 9999:
            mean_voltage = raw_voltages.mean()
            raw_voltages = np.zeros(10000)
            num_samples = 0
            rospy.loginfo('[sharp_sensor_cal] Done with calibration')
        num_samples += 1
        rate.sleep()

   
