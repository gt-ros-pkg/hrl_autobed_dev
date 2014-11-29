#!/usr/bin/env python
import sys
import operator
import numpy as np
import matplotlib.pyplot as plt

import roslib; roslib.load_manifest('autobed_pose_estimator')
import rospy
import cPickle as pkl
from hrl_lib.util import save_pickle, load_pickle
import pressure_map_generator
from sttr_msgs.msg import RagdollObjectArray


class PoseTester():
    '''Gets the dictionary of poses from the training database, collects the current pose of the model, and identifies it as one of the poses in the database'''

    def __init__(self, training_database_file):
        '''Initializes an object of the type PoseGenerator, which can be used to get the present pressure map. Will also initialize the datastructure and the pickle file stuff. Will read the pose_list pickle file to get the joint positions corresponding to the pose_ID'''
        self.training_database_file = training_database_file #This is the training database file we are creating
        self.pressure_mat = pressure_map_generator.MapGenerator()#Create a pressure mat object   
        self.current_pressure_map = np.zeros(self.pressure_mat.get_mat_size_in_taxels()) #Zero out the present pressure map. We will populate this in a bit.
        self.correlated_pressure_val = {}

        try:
            self.training_database = pkl.load(open(self.training_database_file, "rb"))#Load the database of trained poses
        except:
            print "ERROR: No database found at location specified by you. Please correct location and try again"
 
    def pressure_map_correlation(self):
        '''Calls the get_pressure_map() method given by the MapGenerator class until the pressure map is rich enough. Then we will correlate the pressure map with every map in the dictionary. The poseID with the maximum value of normalized correlation is selected as the correct pose.'''

        num_pressure_points = 0 #Variable representing the number of pressure points recorded
        while num_pressure_points<150 and (not rospy.is_shutdown()):
            pressure_map_matrix = self.pressure_mat.get_pressure_map() #Get the current pressure mat readings
            '''Note: Since, we are using a Gazebo based pressure mat simulation for our training, we don't get a very rich pressure map within one reading. Thus we will logically OR our previous pressure map reading with the current one. Since we are not moving in the bed, this is fine.'''

            self.current_pressure_map = np.logical_or(self.current_pressure_map, pressure_map_matrix) #Logical OR as explained above
            num_pressure_points = np.count_nonzero(self.current_pressure_map) #Number of pressure points recorded till now
            print num_pressure_points

            self.pressure_mat.visualize_pressure_map(self.current_pressure_map)

        '''We have obtained a pressure map that is rich enough. Lets correlate that with each of the training poses'''
        for pose_ID in range(1, len(self.training_database)+1):
            self.correlated_pressure_val[pose_ID] = np.correlate(np.ravel(self.training_database[pose_ID]['pressure_map'].astype(int)) , np.ravel(self.current_pressure_map.astype(int))) 
        #Find the pose_ID that corresponds to the maximum correlation 
        max_pose_ID = max(self.correlated_pressure_val.iteritems(), key=operator.itemgetter(1))[0]
        print self.correlated_pressure_val
        return max_pose_ID

if __name__ == "__main__":
    #Initialize pose trainer with a pose_ID
    pose_estimator_trainer = PoseTester(sys.argv[1])
    best_pose = pose_estimator_trainer.pressure_map_correlation()
    print "Best pose is: {}".format(best_pose)