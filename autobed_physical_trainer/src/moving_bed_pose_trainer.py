#!/usr/bin/env python
import sys
import operator
import numpy as np

import roslib; roslib.load_manifest('autobed_physical_trainer')
import rospy
import cPickle as pkl
from std_msgs.msg import Bool
from hrl_msgs.msg import FloatArrayBare
from geometry_msgs.msg import TransformStamped


class BagfileToPickle():
    '''Converts pressure map bagfile to a pickle file with labels'''
    def __init__(self, filename):
        self.mat_pose_sampled = False
        self.ok_to_read_pose = False
        self.filename = filename

        rospy.init_node('pose_trainer_moving_bed', anonymous=True)
        rospy.Subscriber("/fsascan", FloatArrayBare, 
                self.current_physical_pressure_map_callback)
        rospy.Subscriber("/mat_o/pose", TransformStamped,
                self.mat_origin_callback)
        rospy.Subscriber("/head_o/pose", TransformStamped,
                self.head_origin_callback)
        rospy.Subscriber("/torso_o/pose", TransformStamped,
                self.torso_origin_callback)
        rospy.Subscriber("/l_elbow_o/pose", TransformStamped,
                self.l_elbow_origin_callback)
        rospy.Subscriber("/r_elbow_o/pose", TransformStamped,
                self.r_elbow_origin_callback)
        rospy.Subscriber("/l_hand_o/pose", TransformStamped,
                self.l_hand_origin_callback)
        rospy.Subscriber("/r_hand_o/pose", TransformStamped,
                self.r_hand_origin_callback)
        rospy.Subscriber("/l_knee_o/pose", TransformStamped,
                self.l_knee_origin_callback)
        rospy.Subscriber("/r_knee_o/pose", TransformStamped,
                self.r_knee_origin_callback)
        rospy.Subscriber("/l_ankle_o/pose", TransformStamped,
                self.l_ankle_origin_callback)
        rospy.Subscriber("/r_ankle_o/pose", TransformStamped,
                self.r_ankle_origin_callback)
        rospy.Subscriber("/abdout0", FloatArrayBare,
                self.autobed_angle_callback) 
        rospy.Subscriber("/abdstatus0", Bool,
                self.autobed_status_callback) 

        self.abdin0 = rospy.Publisher("/abdin0", FloatArrayBare)
        try:
            self.training_database = pkl.load(open(self.filename, 'rb'))
        except:
            print "Pickle file didn't exist. Creating new pickle dataset."
            self.training_database = {}
        self.count = 0 #When to sample the mat_origin

        self.autobed_pose = []
        self.head_pose = []
        #self.head_orientation = []
        self.torso_pose = []
        self.l_elbow_pose = []
        self.r_elbow_pose = []
        self.l_hand_pose = []
        self.r_hand_pose = []
        self.l_knee_pose = []
        self.r_knee_pose = []
        self.l_ankle_pose = []
        self.r_ankle_pose = []


    def autobed_angle_callback(self, data):
        '''This callback is used to store autobed angles'''
        self.autobed_pose = data.data


    def autobed_status_callback(self, data):
        '''This callback is used to sample data when the autobed reaches a
        certain configuration. We will need only head and legs angles'''

        if data.data == True:
            self.ok_to_read_pose = True
            return
        else:
            return


    def current_physical_pressure_map_callback(self, data):
        '''This callback accepts incoming pressure map from 
        the Vista Medical Pressure Mat and sends it out. 
        Remember, this array needs to be binarized to be used'''
        self.pressure_map  = data.data
        self.ok_to_read_pose = True


    def mat_origin_callback(self, data):
        '''This callback will sample data until its asked to stop'''
        if not self.mat_pose_sampled:
            self.mat_pose = [data.transform.translation.x,
                             data.transform.translation.y,
                             data.transform.translation.z]


    def head_origin_callback(self, data):
        '''This callback will sample data until its asked to stop'''
        self.head_pose = [data.transform.translation.x,
                         data.transform.translation.y,
                         data.transform.translation.z]
        #self.head_orientation = [data.transform.rotation.x,
                                #data.transform.rotation.y,
                                #data.transform.rotation.z,
                                #data.transform.rotation.w]

    def torso_origin_callback(self, data):
        '''This callback will sample data until its asked to stop'''
        self.torso_pose = [data.transform.translation.x,
                         data.transform.translation.y,
                         data.transform.translation.z]


    def l_elbow_origin_callback(self, data):
        '''This callback will sample data until its asked to stop'''
        self.l_elbow_pose = [data.transform.translation.x,
                         data.transform.translation.y,
                         data.transform.translation.z]


    def r_elbow_origin_callback(self, data):
        '''This callback will sample data until its asked to stop'''
        self.r_elbow_pose = [data.transform.translation.x,
                         data.transform.translation.y,
                         data.transform.translation.z]


    def l_hand_origin_callback(self, data):
        '''This callback will sample data until its asked to stop'''
        self.l_hand_pose = [data.transform.translation.x,
                         data.transform.translation.y,
                         data.transform.translation.z]


    def r_hand_origin_callback(self, data):
        '''This callback will sample data until its asked to stop'''
        self.r_hand_pose = [data.transform.translation.x,
                         data.transform.translation.y,
                         data.transform.translation.z]

    def l_knee_origin_callback(self, data):
        '''This callback will sample data until its asked to stop'''
        self.l_knee_pose = [data.transform.translation.x,
                         data.transform.translation.y,
                         data.transform.translation.z]


    def r_knee_origin_callback(self, data):
        '''This callback will sample data until its asked to stop'''
        self.r_knee_pose = [data.transform.translation.x,
                         data.transform.translation.y,
                         data.transform.translation.z]

    def l_ankle_origin_callback(self, data):
        '''This callback will sample data until its asked to stop'''
        self.l_ankle_pose = [data.transform.translation.x,
                         data.transform.translation.y,
                         data.transform.translation.z]


    def r_ankle_origin_callback(self, data):
        '''This callback will sample data until its asked to stop'''
        self.r_ankle_pose = [data.transform.translation.x,
                         data.transform.translation.y,
                         data.transform.translation.z]

    def take_bed_through_one_motion_cycle(self):
        '''This function moves the bed through a certain number of discrete
        steps'''
        head_angle_increment = 15
        leg_angle_increment = 15
        head_angle = 0
        leg_angle = 0
        r = rospy.Rate(1) #1Hz
        while head_angle <= 90: 
            self.abdin0.publish([head_angle, 0, 0])
            r.sleep()
            if self.ok_to_read_pose == True:
                X = (self.pressure_map + tuple(self.autobed_pose))
                self.training_database[X] = (self.head_pose +
                    #self.torso_pose +
                    #self.l_elbow_pose + self.r_elbow_pose + 
                    self.l_hand_pose + self.r_hand_pose + 
                    self.l_knee_pose + self.r_knee_pose +
                    self.l_ankle_pose + self.r_ankle_pose )
                    #+ self.head_orientation)
                self.ok_to_read_pose = False
                head_angle += head_angle_increment

        while leg_angle <= 60:
            self.abdin0.publish([0, 0, leg_angle])
            r.sleep()
            if self.ok_to_read_pose == True:
                X = (self.pressure_map + tuple(self.autobed_pose))
                self.training_database[X] = (self.head_pose +
                    self.torso_pose +
                    self.l_elbow_pose + self.r_elbow_pose + 
                    self.l_hand_pose + self.r_hand_pose + 
                    self.l_knee_pose + self.r_knee_pose +
                    self.l_ankle_pose + self.r_ankle_pose )
                    #+ self.head_orientation)
                self.ok_to_read_pose = False
                head_angle += head_angle_increment
        return


    def run(self):
        '''This code just collects the first 1200 samples of the 
        pressure mat that will come in through the bagfile and
        will label them with the label'''
        raw_input("Press Enter to Start Experiment")
        print "Starting Experiment"
        while not rospy.is_shutdown():
            raw_input("Hit Enter when user is ready with the next pose")
            self.take_bed_through_one_motion_cycle()
        pkl.dump(self.training_database, open(self.filename, "wb"))
                 

if __name__ == "__main__":
    convertor = BagfileToPickle(sys.argv[1])
    convertor.run()
