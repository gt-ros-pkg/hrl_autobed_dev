#!/usr/bin/env python
import sys
import operator
import numpy as np

import roslib; roslib.load_manifest('autobed_physical_trainer')
import rospy
import cPickle as pkl
import math
from hrl_msgs.msg import FloatArrayBare
from geometry_msgs.msg import TransformStamped

NUMOFTAXELS_X = 64#73 #taxels
NUMOFTAXELS_Y = 27#30 
TOTAL_TAXELS = NUMOFTAXELS_X*NUMOFTAXELS_Y

class BagfileToPickle():
    '''Converts pressure map bagfile to a pickle file with labels'''
    def __init__(self, filename):
        self.mat_pose_sampled = False
        self.ok_to_read_pose = False
        self.filename = filename

        rospy.init_node('bag_to_pkl', anonymous=False)
        rospy.Subscriber("/fsascan", FloatArrayBare, 
                self.current_physical_pressure_map_callback)
        rospy.Subscriber("/head_o/pose", TransformStamped,
                self.head_origin_callback)

        try:
            self.head_dataset = pkl.load(open(self.filename, 'rb'))
        except:
            print "Pickle file didn't exist. Creating new pickle dataset."
            self.head_dataset = {}
        self.count = 0 #When to sample the mat_origin
        self.head_orientation = []



    def current_physical_pressure_map_callback(self, data):
        '''This callback accepts incoming pressure map from 
        the Vista Medical Pressure Mat and sends it out. 
        Remember, this array needs to be binarized to be used'''
        if len(data.data) == TOTAL_TAXELS:
            self.pressure_map  = data.data
            self.ok_to_read_pose = True
        else:
            self.ok_to_read_pose = False
            return


    def head_origin_callback(self, data):
        '''This callback will sample data until its asked to stop'''
        q = [data.transform.rotation.x,
             data.transform.rotation.y,
             data.transform.rotation.z,
             data.transform.rotation.w]
        head_orientation_rad = np.array([math.atan2(2*(q[0]*q[1] + q[2]*q[3]), (1 - 2*(q[1]**2 + q[2]**2))),
                                math.asin(2*(q[0]*q[2] - q[3]*q[1])),
                                math.atan2(2*(q[0]*q[3] + q[1]*q[2]), (1 - 2*(q[2]**2 + q[3]**2)))])
        self.head_orientation = head_orientation_rad*(180.0/math.pi)



    def run(self):
        '''This code just collects the first 1200 samples of the 
        pressure mat that will come in through the bagfile and
        will label them with the label'''
        while not rospy.is_shutdown():
            if self.ok_to_read_pose == True and np.size(self.head_orientation)==3:
                if (self.head_orientation[1] < -20): 
                    #This is to make sure the first pose is sampled
                    self.head_dataset[self.pressure_map] = 'right'
                    print 'right'
                elif (self.head_orientation[1] > 20):
                    self.head_dataset[self.pressure_map] = 'left'
                    print 'left'
                else:
                    self.head_dataset[self.pressure_map] = 'center'
                    print 'center'
                self.ok_to_read_pose = False
        pkl.dump(self.head_dataset, open(self.filename, "wb"))
                 

if __name__ == "__main__":
    convertor = BagfileToPickle(sys.argv[1])
    convertor.run()
