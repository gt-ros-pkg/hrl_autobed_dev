#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt

import roslib; roslib.load_manifest('autobed_pose_estimator')
import rospy
import cPickle as pkl
from hrl_msgs.msg import FloatArrayBare
from hrl_lib.util import save_pickle, load_pickle
import pressure_map_generator

BINARIZING_THRESHOLD  = 1

class PoseTrainer():
    '''Gets the current pose ID, and then build a dictionary with the pose ID, 
    pointing to a numpy array corresponding to the pressure map of the pose, 
    and a list corresponding to the joint positions, which is placed in 
    another pickle file'''

    def __init__(self, numposes, training_database_file):
        '''Initializes an object of the type PoseGenerator, which can be used 
        to get the present pressure map. Will also initialize the datastructure 
        and the pickle file stuff. Will read the pose_list pickle file to get 
        the joint positions corresponding to the pose_ID'''
        #This is the training database file we are creating
        self.training_database_file = training_database_file 
        #Current pose, that we are training the system for
        self.numposes = numposes 
        #Create a pressure mat object   
        self.pressure_mat = pressure_map_generator.MapGenerator()
        #Zero out the present pressure map. We will populate this in a bit.
        self.analog_map = np.zeros(self.pressure_mat.get_mat_size_in_taxels()) 
        #Database doesn't exist yet. So we will create it.
        self.training_database = {} 
        #Save an empty dict for now
        pkl.dump(self.training_database, open(self.training_database_file, "wb")) 

        #Start subscribers to accept the incoming human subjects coordinates 
        (rospy.Subscriber("/fsascan", FloatArrayBare, 
            self.current_physical_pressure_map_callback))
        print "###############################################################"
        print "##### WELCOME TO THE AUTOBED PRESSURE COLLECTION PROGRAM #####"
        print "###############################################################"
        print "..."
        print ".."
        print "."
        print "We will be testing {} poses today".format(self.numposes)
        print ("Please refer to the sheet provided for the pose you are going" 
        " to test")
        print "."
        print ".."
        print "..."
        rospy.sleep(30.)


    def binarize_pressure_map(self):
        '''Thresholds the physical pressure map,
        in order to convert it to a binary map'''
        return (np.where(self.analog_map >= BINARIZING_THRESHOLD, 1, 0))


    def begin_training(self):
        '''Will collect a snapshot of the pressure map 20 seconds after the 
        user commands it to collect data uusing the [y] key.
        Once this is done, the map along with the list of joints is safely 
        stored in the training database pickle file'''
        pose_ID = 1
        while pose_ID <= self.numposes:
            #Add the pose_ID key to the dictionary 'training database' and 
            #assign an empty dictionary to it.
            if not self.training_database.has_key(pose_ID):
                self.training_database[pose_ID] = {}
            print "###########################################################"
            print "PREPARE FOR POSE {}".format(pose_ID)
            print "###########################################################"
            print ("YOU HAVE 30 seconds to prep yourself. Look at the chart"
            "for positioning yourself")
            rospy.sleep(20.)
            print "10 Seconds left"
            rospy.sleep(5.)
            print "Clicking snapshot in 5 seconds"
            rospy.sleep(5.)
            print "SNAPSHOT CLICKED!!!" 
            '''Binarize incoming pressure map'''
            current_pressure_map = self.binarize_pressure_map()
            self.pressure_mat.visualize_pressure_map(current_pressure_map)
            print ("Hit [y] if you are satisfied with the picture. "
            "Else Hit the [n] key and we will retake")
            user_ip = raw_input("(y/n):")
            if user_ip == "y":
                #Save the current pose and move on to the next one
                self.training_database[pose_ID]['pressure_map'] = current_pressure_map
                print "Pose number {} SUCCESSFULLY CAPTURED.".format(pose_ID)
                pose_ID += 1
            else:
                print ("CURRENT DATA ERASED. RECLICKING PICTURE FOR THE SAME "
                "POSE {}".format(pose_ID))
                continue

        #Save the database
        pkl.dump(self.training_database, open(self.training_database_file, "wb"))
    

    def current_physical_pressure_map_callback(self, data):
        '''This callback accepts incoming pressure map from 
        the Vista Medical Pressure Mat and sends it out. 
        Remember, this array needs to be binarized to be used'''
        self.analog_map = (np.resize(np.asarray(data.data), 
                        tuple(self.pressure_mat.get_mat_size_in_taxels())))


if __name__ == "__main__":
    #Initialize pose trainer with a pose_ID
    pose_estimator_trainer = PoseTrainer(int(sys.argv[1]), sys.argv[2])
    pose_estimator_trainer.begin_training()

