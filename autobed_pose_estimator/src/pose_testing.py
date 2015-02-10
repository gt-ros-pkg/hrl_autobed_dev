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
from hrl_msgs.msg import FloatArrayBare
from scipy import ndimage

BINARIZING_THRESHOLD = 1 

class PoseTester():
    '''Gets the dictionary of poses from the training database, 
    collects the current pose of the model, and identifies it 
    as one of the poses in the database'''

    def __init__(self, training_database_file, test_type):
        '''Initializes an object of the type PoseGenerator, 
        which can be used to get the present pressure map. 
        Will also initialize the datastructure and the 
        pickle file stuff. Will read the pose_list pickle 
        file to get the joint positions corresponding to the pose_ID'''
        self.training_database_file = training_database_file #This is the training database file we are creating
        self.pressure_mat = pressure_map_generator.MapGenerator()#Create a pressure mat object   
        self.current_pressure_map = np.zeros(self.pressure_mat.get_mat_size_in_taxels()) #Zero out the present pressure map. We will populate this in a bit.
        self.correlated_pressure_val = {}
        self.cog_adjustment_window_size = 5 #Window size for fine adjustment of the cog 
        self.test_type = test_type
        try:
            self.training_database = pkl.load(open(self.training_database_file, "rb"))#Load the database of trained poses
        except:
            print "ERROR: No database found at location specified by you. Please correct location and try again" 
        if test_type == 'gazebo':
            # If the test subject is going to be a Gazebo Green Kevin on the Gazebo Autobed.
            #Start subscribers to accept the incoming human subjects coordinates 
            rospy.Subscriber("/gazebo/ragdollcog/", RagdollObjectArray, self.ragdoll_cog_collection_callback)
 
            self.ok_to_pull_human_coordinates = 0
            self.ground_truth = [] #Variable to hold the ground truth of the coordinates.
        elif test_type == 'physical':
            #If the test is conducted on a physical mat with a real human being
            #Start subscribers to accept the incoming human subjects pressure map 
            rospy.Subscriber("/fsascan", FloatArrayBare, self.current_physical_pressure_map_callback)
            self.physical_pressure_map = np.zeros(self.pressure_mat.get_mat_size_in_taxels()) #Zero out the present pressure map. We will populate this in a bit.
            self.body_cog_map = np.zeros(self.pressure_mat.get_mat_size_in_taxels()) 

    def current_physical_pressure_map_callback(self, data):
        '''This callback accepts incoming pressure map from 
        the Vista Medical Pressure Mat and sends it out. 
        Remember, this array needs to be binarized to be used'''
        self.physical_pressure_map = np.resize(np.asarray(data.data), tuple(self.pressure_mat.get_mat_size_in_taxels()))


    def pressure_map_correlation_gazebo(self):
        '''Calls the get_pressure_map() method given 
        by the MapGenerator class until the pressure 
        map is rich enough. Then we will correlate 
        the pressure map with every map in the 
        dictionary. The poseID with the maximum 
        value of normalized correlation is selected 
        as the correct pose.'''

        num_pressure_points = 0 #Variable representing the number of pressure points recorded
        while num_pressure_points<230 and (not rospy.is_shutdown()):
            pressure_map_matrix = self.pressure_mat.get_pressure_map() #Get the current pressure mat readings
            '''Note: Since, we are using a Gazebo based pressure mat 
            simulation for our training, we don't get a very rich pressure 
            map within one reading. Thus we will logically OR our previous 
            pressure map reading with the current one. Since we are not moving 
            in the bed, this is fine.'''

            self.current_pressure_map = np.logical_or(self.current_pressure_map, pressure_map_matrix) #Logical OR as explained above
            num_pressure_points = np.count_nonzero(self.current_pressure_map) #Number of pressure points recorded till now
            if num_pressure_points >= 150:
                self.ok_to_pull_human_coordinates = self.ok_to_pull_human_coordinates + 1

            self.pressure_mat.visualize_pressure_map(self.current_pressure_map)
            print num_pressure_points

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


    def binarize_pressure_map(self):
        '''Thresholds the physical pressure map,
        in order to convert it to a binary map'''
        return np.where(self.physical_pressure_map>= BINARIZING_THRESHOLD, 1, 0)


    def shift_physical_map(self, pose_ID):
        '''Shifts the pressure map so that alignment of
        the database maps and physical map is possible during
        correlation'''
        database_head_location = np.transpose(np.nonzero(self.training_database[pose_ID]['pressure_map']))[0]
        try:
            physical_head_location = np.transpose(np.nonzero(self.current_pressure_map))[0]
        except IndexError:
            return self.current_pressure_map
        head_shift = database_head_location - physical_head_location
        shifted_pressure_map = np.roll(self.current_pressure_map, head_shift[0], axis=0)
        if head_shift[0]>0:
            shifted_pressure_map[:head_shift[0], :] = 0
        elif head_shift[0]<0:
            shifted_pressure_map[head_shift[0]:, :] = 0
        
        shifted_pressure_map = np.roll(shifted_pressure_map, head_shift[1], axis=1)
        if head_shift[1]>0:
            shifted_pressure_map[:, :head_shift[1]] = 0
        elif head_shift[1]<0:
            shifted_pressure_map[:, head_shift[1]:] = 0

        return shifted_pressure_map


    def pressure_map_correlation_physical(self):
        '''Runs an infinite loop until node is shutdown. 
        At a rate of 1Hz, it takes in the latest pressure map 
        and performs a correlation with the database, 
        it will update the co-ordinates of the limb COGs and will
        present an appropriate visualization of the pressure 
        map and the COGs'''
        rate = rospy.Rate(1) #1Hz rate
        max_pose_ID = 0
        while not rospy.is_shutdown():
            '''Binarize incoming pressure map'''
            self.current_pressure_map = self.binarize_pressure_map()
            #pkl.dump( self.current_pressure_map, open( "pressure_snapshot.p", "wb" ) )
            '''Correlation procedure'''
            for pose_ID in range(1, len(self.training_database)+1):
                try:
                    #Shift current pressure map first so that the heads align
                    shifted_pressure_map = self.shift_physical_map(pose_ID)
                    self.correlated_pressure_val[pose_ID] = np.correlate(np.ravel(self.training_database[pose_ID]['pressure_map'].astype(int)) , np.ravel(shifted_pressure_map.astype(int))) 
                except KeyError:
                    #This pose was probably not stored in the database. No problems. We ignore this and move on.
                    print "WARNING:[Pose Estimation]Pose number {} has not been recorded. Ignoring and moving on..".format(pose_ID)
                    continue

            pkl.dump(self.correlated_pressure_val,
            open("./database/correlated_physical_supine.p", "wb"))

            #Find the pose_ID that corresponds to the maximum correlation 
            max_pose_ID = max(self.correlated_pressure_val.iteritems(), key=operator.itemgetter(1))[0]
            print max_pose_ID

            '''Comment one of the two lines below based on what you want to visualize
            self.training_database will visualize the closest training image
            self.current_pressure_map will visualize the realtime test map'''
            self.pressure_mat.visualize_pressure_map(self.training_database[max_pose_ID]['pressure_map'])
            #self.pressure_mat.visualize_pressure_map(self.current_pressure_map)
            ''' We will now compute the COGs of the links using fine adjustment
            and will display the same in a figure'''
            '''
            tuple_of_centers = self.cog_adjustment(max_pose_ID)
            for i,j in tuple_of_centers:
                self.body_cog_map.itemset((i, j), 1) 

            #self.pressure_mat.visualize_pressure_map(self.body_cog_map)
            self.body_cog_map[self.body_cog_map > 0] = 0
            '''
            rate.sleep()


    def cog_adjustment(self, pose_ID):
        '''Gets a certain pose ID, looks through the corresponding 
        CoG values, and forms a CoG updater window around the 
        present cog positions on the pressure mat. 
        Once this is done, applies the window to current_pressure_map
        , and computes new CoG'''
        correct_coordinate_centers = []
        correct_taxel_centers = []
        current_data = self.training_database[pose_ID]
        #Convert the training data centers to taxels
        coordinate_centers = zip(current_data['centers_x'], current_data['centers_y'])
        taxel_centers = [self.pressure_mat.tuple_coordinates_to_taxel_positions(coordinate_center) for coordinate_center in coordinate_centers]
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
            correct_taxel_center = tuple(element1 + element2 for element1,element2 in zip(center_num, com_window))#Add the current center to the center of mass of the window
            correct_coordinate_centers.append(self.pressure_mat.tuple_taxel_positions_to_coordinates(correct_taxel_center))
            correct_taxel_centers.append(correct_taxel_center) 

        if self.test_type == 'gazebo':
            individual_point_errors = [np.linalg.norm(np.asarray(correct_coordinate_centers[elem]) - np.asarray(self.ground_truth[elem])) for elem in range(len(correct_coordinate_centers))]
            total_measurement_error = sum(individual_point_errors)/len(individual_point_errors)
            return correct_coordinate_centers, self.ground_truth, total_measurement_error
        else:
            return correct_taxel_centers


    def ragdoll_cog_collection_callback(self, data):
        '''We use this method to ge the ground truth. 
        Accepts incoming coordinates from the gazebo ragdoll, 
        and stores it into a variable local to our namespace'''
        if self.ok_to_pull_human_coordinates == 1:
            c_x = np.asarray(data.centers_x)
            c_y = np.asarray(data.centers_y)
            self.ground_truth = zip(c_x, c_y)
            self.ok_to_pull_human_coordinates = self.ok_to_pull_human_coordinates + 1
        else:
            return


if __name__ == "__main__":
    #Initialize pose trainer with a pose_ID

    training_database_file = sys.argv[1] #Where is the training database is located
    test_results_file = sys.argv[2] #Where the testing results are to be stored
    test_type = sys.argv[3] #Whether we are testing with a 'physical' or 'simulated' pressure mat

    pose_estimator = PoseTester(training_database_file, test_type)
    if test_type == 'gazebo':
        best_pose = pose_estimator.pressure_map_correlation_gazebo()
        correct_coordinates, ground_truth, total_error = pose_estimator.cog_adjustment(best_pose)
        print "Correct:"
        print correct_coordinates
        print "Ground Truth:"
        print ground_truth
        print "Average error in measurement:"
        print total_error
        print "Best pose is: {}".format(best_pose)
        
        try:
            test_results = pkl.load(open(test_results_file, "rb"))#Load the database of trained poses
        except:
            test_results = []
            pkl.dump(test_results, open(test_results_file, "wb")) #Save an empty list for now

        current_test_result = {'best_pose':best_pose, 'estimated_positions':correct_coordinates, 'ground_truth':ground_truth, 'average error': total_error}
        test_results.append(current_test_result)
        pkl.dump(test_results, open(test_results_file, "wb")) #save the database

    else:
        pose_estimator.pressure_map_correlation_physical()

