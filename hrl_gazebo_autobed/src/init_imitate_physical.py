#!/usr/bin/env python

import roslib; roslib.load_manifest('hrl_gazebo_autobed')
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from hrl_msgs.msg import FloatArrayBare
from math import pi
from scipy.signal import remez
from scipy.signal import lfilter
import numpy as np

POSE_DICT = {'/autobed_head_controller':0, '/autobed_height_controller':1,
        '/autobed_legs_controller':2, '/autobed_passive_joint_controller':2}

class JointTrajectoryTest():
    def __init__(self, controller='/autobed_height_controller'):
        self.controller = controller
        self.goal_pub = rospy.Publisher(controller+'/command', JointTrajectory)
        self.state_sub = rospy.Subscriber(controller+'/state', 
                JointTrajectoryControllerState, self.state_cb)
        self.joint_names = None
        self.physical_sub = rospy.Subscriber('/abdout0', FloatArrayBare, 
                self.init_physical)
        #Low pass filter design
        self.bin_numbers = 5
        self.bin_numbers_for_leg_filter = 21
        self.collated_cal_angle = np.zeros((self.bin_numbers, 1))
        self.collated_cal_angle_for_legs = np.zeros((
            self.bin_numbers_for_leg_filter, 1))
        self.filtered_angle = None
        self.lpf = remez(self.bin_numbers, [0, 0.1, 0.25, 0.5], [1.0, 0.0])
        self.lpf_for_legs = remez(self.bin_numbers_for_leg_filter, 
                [0, 0.0005, 0.1, 0.5], [1.0, 0.0])


    def init_physical(self, data):
        '''Will initialize gazebo autobed to the actual autobed's intial angles
        '''
        init_angle = data.data[POSE_DICT[self.controller]] 
        if (self.controller =='/autobed_legs_controller') or (self.controller
        =='/autobed_passive_joint_controller' ):
            self.collated_cal_angle_for_legs = np.delete(
                    self.collated_cal_angle_for_legs, 0)
            self.collated_cal_angle_for_legs = np.append(
                    self.collated_cal_angle_for_legs, [self.physical_to_gazebo(init_angle)])
        else:
            self.collated_cal_angle = np.delete(self.collated_cal_angle, 0)
            self.collated_cal_angle = np.append(self.collated_cal_angle, 
                                [self.physical_to_gazebo(init_angle)])
        self.filtered_angle = self.filter_data() 


    def physical_to_gazebo(self, data):
        '''Convert angles reported by the autobed into the angles that the 
        gazebo autobed can read'''
        if self.controller=='/autobed_height_controller':
            cal_angle = data/100
        elif self.controller=='/autobed_head_controller':
            cal_angle = (data*pi/180 - 0.1)
        else:
            cal_angle = (data*pi/180 - 0.1)
        return cal_angle

        
    def state_cb(self, state_msg):
        if self.joint_names is None:
            self.joint_names = state_msg.joint_names


    def filter_data(self):
        '''Creates a low pass filter to filter out high frequency noise'''
        if self.controller=='/autobed_legs_controller':
            filt_data = self.truncate(np.dot(self.lpf_for_legs,
                    self.collated_cal_angle_for_legs))
        elif self.controller=='/autobed_passive_joint_controller':
            filt_data = -(1+(4.0/9.0))*self.truncate(np.dot(self.lpf_for_legs,
                    self.collated_cal_angle_for_legs))
        else: 
            filt_data = np.dot(self.lpf, self.collated_cal_angle)

        return filt_data


    def truncate(self, f):
        '''Truncates/pads a float f to 1 decimal place without rounding'''
        fl_as_str = "%.2f" %f
        return float(fl_as_str)


    def up_msg(self):
        jtm = JointTrajectory()
        jtm.joint_names = self.joint_names
        jtp = JointTrajectoryPoint()
        jtp.positions = [self.filtered_angle]*len(self.joint_names)
        print self.controller, self.filtered_angle
        jtp.time_from_start = rospy.Duration(1.0)
        jtm.points = [jtp]
        return jtm


    def run(self):
        while self.joint_names is None:
            print "Waiting for joint state information from %s/state topic" %self.controller
            rospy.sleep(2)
        while self.filtered_angle is None:
            print "Waiting for Physical Autobed Connection with Gazebo Autobed"
            rospy.sleep(2)
    
        print "Received joint state information and angle information from autobed"
        up_msg = self.up_msg()
        self.goal_pub.publish(up_msg)
        return


if __name__=='__main__':
    rospy.init_node('autobed_height_setup')
    JTT = JointTrajectoryTest(controller = '/autobed_height_controller')
    JTT_h = JointTrajectoryTest(controller='/autobed_head_controller')
    JTT_l = JointTrajectoryTest(controller='/autobed_legs_controller')
    JTT_passive = JointTrajectoryTest(controller='/autobed_passive_joint_controller')
    while not rospy.is_shutdown():
        JTT.run()
        JTT_h.run()
        JTT_l.run()
        JTT_passive.run()
        rospy.sleep(2)
