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
from scipy import ndimage


class PoseTester():
    '''Gets the dictionary of poses from the training database, collects the current pose of the model, and identifies it as one of the poses in the database'''

    def __init__(self, training_database_file):
        '''Initializes an object of the type PoseGenerator, which can be used to get the present pressure map. Will also initialize the datastructure and the pickle file stuff. Will read the pose_list pickle file to get the joint positions corresponding to the pose_ID'''
        self.training_database_file = training_database_file #This is the training database file we are creating
        self.pressure_mat = pressure_map_generator.MapGenerator()#Create a pressure mat object   
        self.current_pressure_map = np.zeros(self.pressure_mat.get_mat_size_in_taxels()) #Zero out the present pressure map. We will populate this in a bit.
        self.correlated_pressure_val = {}
        self.cog_adjustment_window_size = 3 #Window size for fine adjustment of the cog 
        try:
            self.training_database = pkl.load(open(self.training_database_file, "rb"))#Load the database of trained poses
        except:
            print "ERROR: No database found at location specified by you. Please correct location and try again"
 
        #Start subscribers to accept the incoming human subjects coordinates 
        rospy.Subscriber("/gazebo/ragdollcog/", RagdollObjectArray, self.ragdoll_cog_collection_callback)
 
        self.ok_to_pull_human_coordinates = 0
        self.ground_truth = [] #Variable to hold the ground truth of the coordinates.

    def pressure_map_correlation(self):
        '''Calls the get_pressure_map() method given by the MapGenerator class until the pressure map is rich enough. Then we will correlate the pressure map with every map in the dictionary. The poseID with the maximum value of normalized correlation is selected as the correct pose.'''

        num_pressure_points = 0 #Variable representing the number of pressure points recorded
        while num_pressure_points<270 and (not rospy.is_shutdown()):
            pressure_map_matrix = self.pressure_mat.get_pressure_map() #Get the current pressure mat readings
            '''Note: Since, we are using a Gazebo based pressure mat simulation for our training, we don't get a very rich pressure map within one reading. Thus we will logically OR our previous pressure map reading with the current one. Since we are not moving in the bed, this is fine.'''

            self.current_pressure_map = np.logical_or(self.current_pressure_map, pressure_map_matrix) #Logical OR as explained above
            num_pressure_points = np.count_nonzero(self.current_pressure_map) #Number of pressure points recorded till now
            print num_pressure_points
            if num_pressure_points >= 150:
                self.ok_to_pull_human_coordinates = self.ok_to_pull_human_coordinates + 1

            self.pressure_mat.visualize_pressure_map(self.current_pressure_map)

        '''We have obtained a pressure map that is rich enough. Lets correlate that with each of the training poses'''
        for pose_ID in range(1, len(self.training_database)+1):
            try:
                self.correlated_pressure_val[pose_ID] = np.correlate(np.ravel(self.training_database[pose_ID]['pressure_map'].astype(int)) , np.ravel(self.current_pressure_map.astype(int))) 
            except KeyError:
                #This pose was probably not stored in the database. No problems. We ignore this and move on.
                print "WARNING:[Pose Estimation]Pose number {} has not been recorded. Ignoring and moving on..".format(pose_ID)
                continue
        #Find the pose_ID that corresponds to the maximum correlation 
        max_pose_ID = max(self.correlated_pressure_val.iteritems(), key=operator.itemgetter(1))[0]

        return max_pose_ID 


    def cog_adjustment(self, pose_ID):
        '''Gets a certain pose ID, looks through the corresponding CoG values, and forms a CoG updater window around the present cog positions on the pressure mat. 
        Once this is done, applies the window to current_pressure_map, and computes new CoG'''
        correct_coordinate_centers = []
        current_data = self.training_database[pose_ID]
        #Convert the training data centers to taxels
        coordinate_centers = zip(current_data['centers_x'], current_data['centers_y'])
        taxel_centers = [self.pressure_mat.coordinates_to_taxel_positions(coordinate_center) for coordinate_center in coordinate_centers]
        #Create a bounding box of 5 taxels around each center 
        for center_num in taxel_centers:
            min_x = (center_num[0] - int(self.cog_adjustment_window_size/2)) if (center_num[0] - int(self.cog_adjustment_window_size/2)) >= 0 else 0
            max_x = (center_num[0] + int(self.cog_adjustment_window_size/2)+1) if (center_num[0] + int(self.cog_adjustment_window_size/2)+1) <= 72 else 72
            min_y = (center_num[1] - int(self.cog_adjustment_window_size/2)) if (center_num[1] - int(self.cog_adjustment_window_size/2)) >= 0 else 0
            max_y = (center_num[1] + int(self.cog_adjustment_window_size/2)+1) if (center_num[1] + int(self.cog_adjustment_window_size/2)+1) <= 29 else 29
            
            #Make the current center taxel 1, otherwise the center of mass wont be a number
            self.current_pressure_map[center_num[0], center_num[1]] = True

            new_center_window = self.current_pressure_map[min_x:max_x, min_y:max_y]
            com_window = ndimage.measurements.center_of_mass(new_center_window.astype(int)) #Compute center of mass of window
            correct_taxel_centers = tuple(element1 + element2 for element1,element2 in zip(center_num, com_window))#Add the current center to the center of mass of the window
            correct_coordinate_centers.append(self.pressure_mat.taxel_positions_to_coordinates(correct_taxel_centers))
        
        individual_point_errors = [np.linalg.norm(np.asarray(correct_coordinate_centers[elem]) - np.asarray(self.ground_truth[elem])) for elem in range(len(correct_coordinate_centers))]
        total_measurement_error = sum(individual_point_errors)/len(individual_point_errors)

        return correct_coordinate_centers, self.ground_truth, total_measurement_error



    def ragdoll_cog_collection_callback(self, data):
        '''We use this method to ge the ground truth. Accepts incoming coordinates from the gazebo ragdoll, and stores it into a variable local to our namespace'''
        if self.ok_to_pull_human_coordinates == 1:
            c_x = np.asarray(data.centers_x)
            c_y = np.asarray(data.centers_y)
            self.ground_truth = zip(c_x, c_y)
            self.ok_to_pull_human_coordinates = self.ok_to_pull_human_coordinates + 1
        else:
            return


if __name__ == "__main__":
    #Initialize pose trainer with a pose_ID
    pose_estimator = PoseTester(sys.argv[1])
    best_pose = pose_estimator.pressure_map_correlation()
    correct_coordinates, ground_truth, total_error = pose_estimator.cog_adjustment(best_pose)
    print "Correct:"
    print correct_coordinates
    print "Ground Truth:"
    print ground_truth
    print "Average error in measurement:"
    print total_error
    print "Best pose is: {}".format(best_pose)
    
    test_results_file = sys.argv[2]
    try:
        test_results = pkl.load(open(test_results_file, "rb"))#Load the database of trained poses
    except:
        test_results = []
        pkl.dump(test_results, open(test_results_file, "wb")) #Save an empty list for now

    current_test_result = {'best_pose':best_pose, 'estimated_positions':correct_coordinates, 'ground_truth':ground_truth, 'average error': total_error}
    test_results.append(current_test_result)
    pkl.dump(test_results, open(test_results_file, "wb")) #save the database
