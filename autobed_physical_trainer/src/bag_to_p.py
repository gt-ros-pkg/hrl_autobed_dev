#!/usr/bin/env python
import sys
import operator
import numpy as np

import roslib; roslib.load_manifest('autobed_physical_trainer')
import rospy
import cPickle as pkl
from hrl_msgs.msg import FloatArrayBare
from geometry_msgs.msg import TransformStamped


class BagfileToPickle():
    '''Converts pressure map bagfile to a pickle file with labels'''
    def __init__(self, filename):
        self.mat_pose_sampled = False
        self.ok_to_read_pose = False
        self.filename = filename

        rospy.init_node('bag_to_pkl', anonymous=False)
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
 
        try:
            self.training_database = pkl.load(open(self.filename, 'rb'))
        except:
            print "Pickle file didn't exist. Creating new pickle dataset."
            self.training_database = {}
        self.count = 0 #When to sample the mat_origin

        self.mat_pose = []
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



    def run(self):
        '''This code just collects the first 1200 samples of the 
        pressure mat that will come in through the bagfile and
        will label them with the label'''
        while not rospy.is_shutdown():
            self.curr_pose = np.asarray([self.head_pose, 
                self.torso_pose, self.l_elbow_pose,self.r_elbow_pose, 
                self.l_hand_pose, self.r_hand_pose, self.l_knee_pose, 
                self.r_knee_pose, self.l_ankle_pose, 
                self.r_ankle_pose])
            if self.ok_to_read_pose == True and np.size(self.curr_pose)==30:
                self.count += 1
                dist_array = []
                #After 20 seconds, we sample mat pose
                if self.count == 100 and not self.mat_pose_sampled:
                    self.training_database['mat_o'] = self.mat_pose
                    self.mat_pose_sampled = True
                    print "Mat pose sampled."
                #This is to make sure the first pose is sampled
                if self.count == 1:
                    self.prev_pose = np.add(np.asarray(self.curr_pose),
                                    5.0*np.ones(np.shape(self.curr_pose)))


                for joint_num in range(len(self.curr_pose)):
                    dist_array.append(np.linalg.norm(
                        np.asarray(self.curr_pose[joint_num]) - 
                        np.asarray(self.prev_pose[joint_num])))
                #If the ground truth between consecutive samples is greater
                #than a certain threshold in Eucledian distance, only then log
                #this sample
                dist_mean = np.mean(dist_array)
                if dist_mean >= 0.01:
                    self.training_database[self.pressure_map] = (
                        self.head_pose + self.torso_pose +
                        self.l_elbow_pose + self.r_elbow_pose + 
                        self.l_hand_pose + self.r_hand_pose + 
                        self.l_knee_pose + self.r_knee_pose +
                        self.l_ankle_pose + self.r_ankle_pose )
                        #+ self.head_orientation)
                    self.prev_pose = self.curr_pose[:]

                self.ok_to_read_pose = False

        pkl.dump(self.training_database, open(self.filename, "wb"))
                 

if __name__ == "__main__":
    convertor = BagfileToPickle(sys.argv[1])
    convertor.run()
