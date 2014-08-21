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

class ProximitySensorDriver(object):
    ''' A class for collecting raw data from a Sharp IR Proximity sensor
    attached to an arduino analog input device.
    '''
    def __init__(self, num_sensors, param_file=None, dev='/dev/ttyUSB0', baudrate=115200):
        self.num_sensors = num_sensors
        self.serial_driver = serial_driver.SerialDriver(self.num_sensors, dev, baudrate)
        self.zero_sensor_sub = rospy.Subscriber('/darpa_arm/zero_prox_sensor', Empty, self.zero_sensor_cb)

        self.frames_service = rospy.Service('/darpa_arm_link0/taxels/srv/local_coord_frames',
                                            None_TransformArray,
                                            self.local_coord_frames_cb)
        self.links_service = rospy.Service('/darpa_arm_link0/taxels/srv/link_name',
                                            None_String,
                                            self.link_name_cb)

        #Initialize arrays for holding sensor data
        #self.cal_coefficients = 1.1214 * np.ones(self.num_sensors) #Default from sensor datasheet
        #self.cal_exponents = -1.072057 * np.ones(self.num_sensors) #Default from sensor datasheet
        
        #Initialize arrays for holding sensor data
        self.cal_coefficients = 10.34297387 * np.ones(self.num_sensors) #Computed from calibration experiments. Will output in centimeters
        self.cal_offsets = -0.20397751 * np.ones(self.num_sensors) #Computed from calibration experiments. Will output in centimeters
        self.cal_exponents = -0.94007202 * np.ones(self.num_sensors) #Computed from calibration experiments. Will output in centimeters
        self.frames = np.zeros(self.num_sensors)
        self.sensor_positions = np.zeros( [3, self.num_sensors] )
        self.sensor_quats =  np.zeros( [4, self.num_sensors] )

        #Optional filtering initializations
        self.current_bin_number = 0
        self.bin_numbers = 21
        self.collated_cal_data = np.zeros((self.bin_numbers, self.num_sensors))
        self.filtered_data = np.zeros(self.num_sensors)

        if param_file is None:
            rospy.logwarn('[prox_sensor_driver] No parameters loaded for Proximity Sensors')
        else:
            sensor_config_data = rosparam.load_file(param_file)[0][0]
            for channel, sensor in sensor_config_data.iteritems():
                self.frames[channel] = sensor['normal_pose']['frame']
                self.cal_coefficients[channel] = sensor['calibration']['coefficient']
                self.cal_offsets[channel] = sensor['calibration']['offset']
                self.cal_exponents[channel] = sensor['calibration']['exponent']
                self.sensor_positions[:,channel] =  sensor['normal_pose']['position']
                self.sensor_quats[:,channel] =  sensor['normal_pose']['orientation']

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
        print "[prox_sensor_driver]: Finished initialization"

    def _read_raw_data(self):
        try:
            raw_data = np.array(self.serial_driver.getValues())
            if any(np.isnan(raw_data)):
                raise ValueError("Raw data from proximity sensor contains NaN's")
        except ValueError as e:
            print "[prox_sensor_driver]: Warning: %s" %e
            raise e
        except:
          print "Reading data in proximity sensor driver failed"
          raise
        assert len(raw_data)==self.num_sensors, "Size of incoming data from proximity sensor does not match number of sensors."
        return raw_data

    def get_sensor_data(self):
        try:
            raw_data = self._read_raw_data()
        except ValueError as e:
            raw_data = np.zeros(self.num_sensors)

        cal_data = self.calibrate_data(raw_data)
        cal_data[np.where(cal_data<0)[0]] = 0.
        
        #Optional filtering.
        if self.current_bin_number == self.bin_numbers:
            self.current_bin_number = 0
            self.filtered_data = self.filter_data()
        else:
            self.collated_cal_data[self.current_bin_number,:] += cal_data
            self.current_bin_number += 1

        return self.frames, self.sensor_positions, self.sensor_quats, raw_data, self.filtered_data

    def calibrate_data(self, raw_data):
        data = raw_data[raw_data<=0.]=1.0E-8
       #return self.cal_coefficients * raw_data ** self.cal_exponents
        return self.cal_coefficients * (self.to_voltage(raw_data) + self.cal_offsets) ** self.cal_exponents

    def filter_data(self):
        '''Creates a low pass filter to filter out high frequency noise'''
        lpf = remez(self.bin_numbers, [0, 0.2, 0.25, 0.5], [1.0, 0.0])
        filt_data = np.zeros(self.num_sensors)
        for i in range(self.num_sensors):
            filt_data[i] = lfilter(lpf, 1, self.collated_cal_data[:,i])

        self.collated_cal_data = np.zeros((self.bin_numbers, self.num_sensors))
        return filt_data

    def local_coord_frames_cb(self, req):
        tar = None_TransformArrayResponse()
        tar.data = [Transform()]*self.num_sensors
        for i in range(self.num_sensors):
            t = Transform()
            t.translation.x = self.sensor_positions[0,i]
            t.translation.y = self.sensor_positions[1,i]
            t.translation.z = self.sensor_positions[2,i]
            #print "translation for ", i, " is :\n", self.sensor_positions[:,i]

            t.rotation.x = self.sensor_quats[0,i]
            t.rotation.y = self.sensor_quats[1,i]
            t.rotation.z = self.sensor_quats[2,i]
            t.rotation.w = self.sensor_quats[3,i]
            tar.data[i] = t
        #print "service call gives :\n", tar
        return tar

    def link_name_cb(self, req):
        return None_StringResponse(data='/torso_lift_link')

    def zero_sensor_cb(self, msg):
        raw_data = self._read_raw_data()
        assert len(raw_data) == len(self.intercepts), "Cannot re-zero proximity sensor, size of current raw data does not match list of intercepts"
        self.intercepts = np.array(raw_data)

    def to_voltage(self, raw_data):
        '''Converts ADC values to voltage readings. Assumes 10 bit ADC, and 5V 
        ADC reference voltage for Vcc = 5Volts'''
        return 5.0*raw_data/1024.0


if __name__=='__main__':
    rospy.init_node('prox_sensor_driver')
    from std_msgs.msg import Float32
    pub0 = rospy.Publisher('/pst0', Float32)
    pub1= rospy.Publisher('/pst1', Float32)
    pub2= rospy.Publisher('/pst2', Float32)
    pub3= rospy.Publisher('/pst3', Float32)
    pub4= rospy.Publisher('/pst4', Float32)
    pub5= rospy.Publisher('/pst5', Float32)
    pub6= rospy.Publisher('/pst6', Float32)
    pub7= rospy.Publisher('/pst7', Float32)
    if len(sys.argv) < 3:
        sys.stderr.write('Usage: rosrun packagename sharp_prox_driver.py  /dev/USBx number_of_sensors param_file_path')
        sys.exit(1)

    prox_driver = ProximitySensorDriver(int(sys.argv[2]), sys.argv[3], dev=sys.argv[1])

    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        data = prox_driver.get_sensor_data()
        raw_data = data[-2]
        cal_data = data[-1]

        pub0.publish(Float32(cal_data[0]))
        pub1.publish(Float32(cal_data[1]))
        pub2.publish(Float32(cal_data[2]))
        pub3.publish(Float32(cal_data[3]))
 
        pub4.publish(Float32(raw_data[0]))
        pub5.publish(Float32(raw_data[1]))
        pub6.publish(Float32(raw_data[2]))
        pub7.publish(Float32(raw_data[3]))
        rate.sleep()
