#!/usr/bin/env python
import sys
import operator
import numpy as np

import roslib; roslib.load_manifest('autobed_pose_estimator')
import rospy
import cPickle as pkl
from hrl_lib.util import save_pickle, load_pickle
from hrl_msgs.msg import FloatArrayBare
from geometry_msgs.msg import TransformStamped

class BagfileToPickle():
    '''Converts pressure map bagfile to a pickle file with labels'''
    def __init__(self, filename):
        self.filename = filename
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/fsascan", FloatArrayBare, self.current_physical_pressure_map_callback)
        rospy.Subscriber("/head_o/pose", TransformStamped,
                self.mat_origin_callback)
        rospy.Subscriber("/mat_o/pose", TransformStamped,
                self.head_origin_callback)
        self.training_database = {}
        self.ok_to_read_pose = False
        self.count = 0 #When to sample the mat_origin
        self.mat_pose_sampled = False
        self.mat_pose = []
        self.head_pose = []

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
                             data.transform.translation.z,
                             data.transform.rotation.x,
                             data.transform.rotation.y,
                             data.transform.rotation.z,
                             data.transform.rotation.w]

    def head_origin_callback(self, data):
        '''This callback will sample data until its asked to stop'''
        self.head_pose = [data.transform.translation.x,
                         data.transform.translation.y,
                         data.transform.translation.z,
                         data.transform.rotation.x,
                         data.transform.rotation.y,
                         data.transform.rotation.z,
                         data.transform.rotation.w]


    def run(self):
        '''This code just collects the first 1200 samples of the 
        pressure mat that will come in through the bagfile and
        will label them with the label'''
        while not rospy.is_shutdown():
            if self.ok_to_read_pose == True:
                self.count += 1
                #After 20 seconds, we sample mat pose
                if self.count == 1000 and not self.mat_pose_sampled:
                    self.training_database['mat_o'] = self.mat_pose
                    self.mat_pose_sampled = True

                self.training_database[self.pressure_map] = self.head_pose
                self.ok_to_read_pose = False

        pkl.dump(self.training_database, open(self.filename, "wb"))
                 

if __name__ == "__main__":
    convertor = BagfileToPickle(sys.argv[1])
    convertor.run()
