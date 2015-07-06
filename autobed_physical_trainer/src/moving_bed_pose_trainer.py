#!/usr/bin/env python
import sys
import operator
import numpy as np
import math
import atexit
from time import sleep
import time

import roslib; roslib.load_manifest('autobed_physical_trainer')
import rospy
import cPickle as pkl
from std_msgs.msg import Bool, String
from hrl_msgs.msg import FloatArrayBare
from geometry_msgs.msg import TransformStamped
from hrl_lib.keyboard_input import KeyboardInput

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
        rospy.Subscriber("/abd_head_angle/pose", TransformStamped, 
                self.autobed_head_angle_cb)
        rospy.Subscriber("/abd_leg_angle/pose", TransformStamped, 
                self.autobed_leg_angle_cb)
        self.abdin0 = rospy.Publisher("/abdin0", FloatArrayBare)
        rospy.Subscriber("/abdout0", FloatArrayBare,
                self.autobed_angle_callback) 
        rospy.Subscriber("/abdstatus0", Bool,
                self.autobed_status_callback) 

        try:
            self.training_database = pkl.load(open(self.filename, 'rb'))
        except:
            print "Pickle file didn't exist. Creating new pickle dataset."
            self.training_database = {}
        self.count = 0 #When to sample the mat_origin
        self.SIZE_OF_CHECK_BUFFER = 20
        self.check_buffer = ([True]*(self.SIZE_OF_CHECK_BUFFER/2) + 
                                [False]*(self.SIZE_OF_CHECK_BUFFER/2))
        self.comparison_buffer = self.check_buffer[:]
        self.reached_goal = False
        self.autobed_pose = []
        self.head_angle = 0
        self.leg_angle = 0

        self.pressure_map = []
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
        self.key_ip = KeyboardInput()
        self.key_pressed = None 

    def keypress(self):
        self.key_pressed = self.key_ip.getch()
        print self.key_pressed              


    def autobed_angle_callback(self, data):
        '''This callback is used to store autobed angles'''
        self.autobed_pose = data.data


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


    def autobed_status_callback(self, data):
        '''This callback is used to sample data when the autobed reaches a
        certain configuration. We will need only head and legs angles'''
        self.check_buffer.append(data.data)
        self.check_buffer.pop(0)
        if self.check_buffer > self.comparison_buffer:
            self.reached_goal = True
        else:
            self.reached_goal = False


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


    def record_data_without_moving_bed(self):
        '''This function starts recording the bed pressure and pose ground 
        truth, without moving the bed.'''
        print "Recording data now..."
        while not rospy.is_shutdown():
            self.keypress()
            if self.key_pressed != 'p':
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
            else:
                print "Pausing Now"
                return


    def take_bed_through_one_motion_cycle(self):
        '''This function moves the bed through a certain number of discrete
        steps'''
        print "Beginning training cycle. Will move head now"
        print "Head Angle at start is {}".format(self.head_angle)
        while self.reached_goal ==True and not rospy.is_shutdown():
            self.abdin0.publish([40.0, float('nan'), float('nan')])
            sleep(2)
        while self.reached_goal == False and not rospy.is_shutdown():
            print "Head Angle:{}".format(self.head_angle)
            if self.ok_to_read_pose == True:
                X = (self.pressure_map + tuple(self.autobed_pose))
                self.training_database[X] = 1
                '''(self.head_pose +
                    #self.torso_pose +
                    #self.l_elbow_pose + self.r_elbow_pose + 
                    self.l_hand_pose + self.r_hand_pose + 
                    self.l_knee_pose + self.r_knee_pose +
                    self.l_ankle_pose + self.r_ankle_pose )
                    #+ self.head_orientation)'''
                self.ok_to_read_pose = False
        print "ASK THE USER TO ADJUST THEMSELF"
        rospy.sleep(10)
        while self.reached_goal ==True and not rospy.is_shutdown():
            self.abdin0.publish([75.0, float('nan'), float('nan')])
            sleep(2)
        while self.reached_goal == False and not rospy.is_shutdown():
            print "Head Angle:{}".format(self.head_angle)
            if self.ok_to_read_pose == True:
                X = (self.pressure_map + tuple(self.autobed_pose))
                self.training_database[X] = 1
                '''(self.head_pose +
                    #self.torso_pose +
                    #self.l_elbow_pose + self.r_elbow_pose + 
                    self.l_hand_pose + self.r_hand_pose + 
                    self.l_knee_pose + self.r_knee_pose +
                    self.l_ankle_pose + self.r_ankle_pose )
                    #+ self.head_orientation)'''
                self.ok_to_read_pose = False

        print "Bringing the head down.."
        print "Head Angle:{}".format(self.head_angle)
        while self.reached_goal == True and not rospy.is_shutdown():
            self.abdin0.publish([0.0, float('nan'), float('nan')])
            sleep(2)
        while self.reached_goal == False and not rospy.is_shutdown():
            continue
        print "Head completely down. Will raise the legs now"
        print "Leg Angle at start is {}".format(self.leg_angle)
        while self.reached_goal ==True and not rospy.is_shutdown():
            self.abdin0.publish([float('nan'), float('nan'), 45.0])
            sleep(2)
        while self.reached_goal == False and not rospy.is_shutdown():
            print "Leg Angle:{}".format(self.leg_angle)
            if self.ok_to_read_pose == True:
                X = (self.pressure_map + tuple(self.autobed_pose))
                self.training_database[X] = 1
                '''(self.head_pose +
                    #self.torso_pose +
                    #self.l_elbow_pose + self.r_elbow_pose + 
                    self.l_hand_pose + self.r_hand_pose + 
                    self.l_knee_pose + self.r_knee_pose +
                    self.l_ankle_pose + self.r_ankle_pose )
                    #+ self.head_orientation)'''
                self.ok_to_read_pose = False
        print "Bringing the legs down.."
        print "Leg Angle:{}".format(self.leg_angle)
        while self.reached_goal == True and not rospy.is_shutdown():
            self.abdin0.publish([float('nan'), float('nan'), 0.0])
            sleep(2)
        while self.reached_goal == False and not rospy.is_shutdown():
            continue
        print "***CURRENT POSE TESTED. MOVE TO NEXT POSE"
        return


    def exit_training(self):
        try:
            while self.reached_goal ==True and not rospy.is_shutdown():
                self.abdin0.publish([float('nan'), float('nan'), float('nan')])
                sleep(2)

            pkl.dump(self.training_database, open(self.filename, "wb"))
            print "Successfully saved data in {}. Exiting now...".format(
                self.filename)
        except:
            print "Couldnt save data to pickle file. Repeat test now!"


    def run(self):
        '''This code just collects the first 1200 samples of the 
        pressure mat that will come in through the bagfile and
        will label them with the label'''
        raw_input("Press Enter to Start Experiment")
        print "Starting Experiment"
        while not rospy.is_shutdown():
            user_ip = str(raw_input(
                "Hit [m/p] when user is ready with the next pose"))
            print user_ip
            if user_ip == 'm':
                self.take_bed_through_one_motion_cycle()
            elif user_ip == 'p':
                print "Hit [p] again to pause recording..."
                self.record_data_without_moving_bed()
            else:
                print "Wrong input."
                print "Enter [m] to move bed."
                print "Enter [p] to start recording data."
        atexit.register(self.exit_training)
                 

if __name__ == "__main__":
    convertor = BagfileToPickle(sys.argv[1])                                         
    convertor.run()
