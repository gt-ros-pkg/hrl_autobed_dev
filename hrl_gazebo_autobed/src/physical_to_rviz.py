#!/usr/bin/env python
import numpy as np
import roslib; roslib.load_manifest('hrl_msgs'); roslib.load_manifest('tf')
import rospy
import tf
from hrl_msgs.msg import FloatArrayBare
from sensor_msgs.msg import JointState
from math import pi
from scipy.signal import remez
from scipy.signal import lfilter


class AutobedConverter():

    def __init__(self):
        rospy.init_node('autobed_state_publisher', anonymous = False)
        self.joint_pub = rospy.Publisher('joint_states', JointState)
        self.sendToRviz=tf.TransformBroadcaster()
        self.listener = tf.TransformListener() 
        rospy.Subscriber("/abdout0", FloatArrayBare, self.pose_callback)
        #Low pass filter design
        self.bed_height = 0
        self.bin_numbers = 5
        self.bin_numbers_for_leg_filter = 21
        self.collated_head_angle = np.zeros((self.bin_numbers, 1))
        self.collated_leg_angle = np.zeros((
            self.bin_numbers_for_leg_filter, 1))
        self.lpf = remez(self.bin_numbers, [0, 0.1, 0.25, 0.5], [1.0, 0.0])
        self.lpf_for_legs = remez(self.bin_numbers_for_leg_filter, 
                [0, 0.0005, 0.1, 0.5], [1.0, 0.0])



    #callback for the pose messages from the autobed
    def pose_callback(self, data): 
        poses=np.asarray(data.data);
        
        self.bed_height = ((poses[1]/100) - 0.09) if (((poses[1]/100) - 0.09) 
                > 0) else 0
        head_angle = (poses[0]*pi/180 - 0.1)
        leg_angle = (poses[2]*pi/180 - 0.1)
        self.collated_head_angle = np.delete(self.collated_head_angle, 0)
        self.collated_head_angle = np.append(self.collated_head_angle,
                [head_angle])
        self.collated_leg_angle = np.delete(self.collated_leg_angle, 0)
        self.collated_leg_angle = np.append(self.collated_leg_angle,
                [leg_angle])
 
    def filter_data(self):
        '''Creates a low pass filter to filter out high frequency noise'''
        self.leg_filt_data = self.truncate(np.dot(self.lpf_for_legs,
                    self.collated_leg_angle))
        self.head_filt_data = np.dot(self.lpf, self.collated_head_angle)
        return


    def truncate(self, f):
        '''Truncates/pads a float f to 1 decimal place without rounding'''
        fl_as_str = "%.2f" %f
        return float(fl_as_str)


    def run (self):
        rate = rospy.Rate(30) #30 Hz

        joint_state = JointState()
        while not rospy.is_shutdown():
            joint_state.header.stamp = rospy.Time.now()
            #Filter data
            self.filter_data()

            joint_state.name = [None]*(10)
            joint_state.position = [None]*(10)
            joint_state.name[0] = "autobed_height_joint"
            joint_state.name[1] = "head_rest_hinge"
            joint_state.name[2] = "leg_rest_upper_joint"
            joint_state.name[3] = "leg_rest_upper_lower_joint"
            joint_state.name[4] = "X"
            joint_state.name[5] = "torso_camera_joint"
            joint_state.name[6] = "mid_body_support"
            joint_state.name[7] = "head_support"
            joint_state.name[8] = "leg_rest_lower_support"
            joint_state.name[9] = "leg_rest_upper_support"
            joint_state.position[0] = self.bed_height
            joint_state.position[1] = self.head_filt_data
            joint_state.position[2] = self.leg_filt_data
            joint_state.position[3] = -(1+(4.0/9.0))*self.leg_filt_data
            joint_state.position[4] = 0
            joint_state.position[5] = 0
            joint_state.position[6] = 0
            joint_state.position[7] = 0
            joint_state.position[8] = 0
            joint_state.position[9] = 0

            self.joint_pub.publish(joint_state)
         
            rate.sleep()
        return  


if __name__ == "__main__":
    a=AutobedConverter()
    a.run()
