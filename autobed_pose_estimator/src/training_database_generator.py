#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt

import roslib; roslib.load_manifest('autobed_pose_estimator')
import rospy
import cPickle as pkl
from hrl_lib.util import save_pickle, load_pickle
import pressure_map_generator
from sttr_msgs.msg import RagdollObjectArray


class PoseTrainer():
    '''Gets the current pose ID, and then build a dictionary with the pose ID, pointing to a numpy array corresponding to the pressure map of the pose, and a list corresponding to the joint positions, which is placed in another pickle file'''

    def __init__(self, pose_ID, training_database_file):
        '''Initializes an object of the type PoseGenerator, which can be used to get the present pressure map. Will also initialize the datastructure and the pickle file stuff. Will read the pose_list pickle file to get the joint positions corresponding to the pose_ID'''
        self.training_database_file = training_database_file #This is the training database file we are creating
        self.pose_ID = pose_ID #Current pose, that we are training the system for
        self.pressure_mat = pressure_map_generator.MapGenerator()#Create a pressure mat object   
        self.current_pressure_map = np.zeros(self.pressure_mat.get_mat_size_in_taxels()) #Zero out the present pressure map. We will populate this in a bit.
        self.ok_to_pull_human_coordinates = 0

        try:
            self.training_database = pkl.load(open(self.training_database_file, "rb"))#Load the database of trained poses
        except:
            self.training_database = {} #Database doesn't exist yet. So we will create it.
            pkl.dump(self.training_database, open(self.training_database_file, "wb")) #Save an empty dict for now
        #Add the pose_ID key to the dictionary 'training database' and assign an empty dictionary to it.
        if not self.training_database.has_key(pose_ID):
            self.training_database[pose_ID] = {}

        #Start subscribers to accept the incoming human subjects coordinates 
        rospy.Subscriber("/gazebo/ragdollcog/", RagdollObjectArray, self.ragdoll_cog_collection_callback)
 
    def begin_training(self):
        '''Calls the get_pressure_map() method given by the MapGenerator class until the pressure map is rich enough to be stored in the database. Once this is done, the map along with the list of joints is safely stored in the training database pickle file'''

        num_pressure_points = 0 #Variable representing the number of pressure points recorded
        while num_pressure_points<280 and (not rospy.is_shutdown()):
            pressure_map_matrix = self.pressure_mat.get_pressure_map() #Get the current pressure mat readings
            '''Note: Since, we are using a Gazebo based pressure mat simulation for our training, we don't get a very rich pressure map within one reading. Thus we will logically OR our previous pressure map reading with the current one. Since we are not moving in the bed, this is fine.'''

            self.current_pressure_map = np.logical_or(self.current_pressure_map, pressure_map_matrix) #Logical OR as explained above
            num_pressure_points = np.count_nonzero(self.current_pressure_map) #Number of pressure points recorded till now
            print num_pressure_points
            if num_pressure_points >= 150:
                self.ok_to_pull_human_coordinates = self.ok_to_pull_human_coordinates + 1

            self.pressure_mat.visualize_pressure_map(self.current_pressure_map)

        '''We have obtained a pressure map that is rich enough. Lets upload that to the database now'''
        self.training_database[self.pose_ID]['pressure_map'] = self.current_pressure_map #assign the current pressure map as a value to the pose_ID key.
        pkl.dump(self.training_database, open(self.training_database_file, "wb")) #save the database
    

    def ragdoll_cog_collection_callback(self, data):
        ''' Accepts incoming coordinates from the gazebo ragdoll, and stores it into a variable local to our namespace'''
        if self.ok_to_pull_human_coordinates == 1:
            self.training_database[self.pose_ID]['frame_names'] = np.asarray(data.frame_names)
            self.training_database[self.pose_ID]['centers_x'] = np.asarray(data.centers_x)
            self.training_database[self.pose_ID]['centers_y'] = np.asarray(data.centers_y)
            self.training_database[self.pose_ID]['centers_z'] = np.asarray(data.centers_z)
            self.training_database[self.pose_ID]['rotation_x'] = np.asarray(data.rotation_x)
            self.training_database[self.pose_ID]['rotation_y'] = np.asarray(data.rotation_y)
            self.training_database[self.pose_ID]['rotation_z'] = np.asarray(data.rotation_z)
            self.training_database[self.pose_ID]['rotation_w'] = np.asarray(data.rotation_w)
            self.ok_to_pull_human_coordinates = self.ok_to_pull_human_coordinates + 1
        else:
            return

if __name__ == "__main__":
    #Initialize pose trainer with a pose_ID
    pose_estimator_trainer = PoseTrainer(int(sys.argv[1]), sys.argv[2])
    pose_estimator_trainer.begin_training()

