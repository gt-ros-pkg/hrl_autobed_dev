#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt

import roslib; roslib.load_manifest('autobed_pose_estimator')
import rospy

from hrl_haptic_manipulation_in_clutter_msgs.msg import TaxelArray 

class PoseTrainer():
    '''Subscribes to the gazebo autobed and displays a map of whatever object is placed on the surface of the bed'''
    def __init__(self):
        '''Node that listens into the Gazebo plugin output data array, which consists of coordinates of the pressure points.
        These coordinates are converted into percentages of the total width/height of the bed. Once this is done, then the pressure points can be used 
        to correlate with a mat of any size and resolution. Once this is done, visualization will be done.'''
        self.head_centers_x = np.array([])
        self.head_centers_y = np.array([])
        self.body_centers_x = np.array([])
        self.body_centers_y = np.array([])
        self.legs_upper_centers_x = np.array([])
        self.legs_upper_centers_y = np.array([])
        self.legs_lower_centers_x = np.array([])
        self.legs_lower_centers_y = np.array([])
        self.width_ratio = np.array([])
        self.height_ratio = np.array([])
        self.bed_width = 0.915 #meters
        self.bed_half_width = self.bed_width/2
        self.bed_height = 2.08 #metres 
        self.numoftaxels_x = 73 #taxels
        self.numoftaxels_y = 30 
        self.pressure_map = np.zeros([self.numoftaxels_x, self.numoftaxels_y])

        #Start subscribers to accept the incoming pressure coordinates 
        rospy.Subscriber("/head_rest_link_pressuremat_sensor/taxels/forces", TaxelArray, self.head_pressure_collection_callback)
        rospy.Subscriber("/torso_lift_link_pressuremat_sensor/taxels/forces", TaxelArray, self.body_pressure_collection_callback)
        rospy.Subscriber("/leg_rest_upper_link_pressuremat_sensor/taxels/forces", TaxelArray, self.legs_upper_pressure_collection_callback)
        rospy.Subscriber("/leg_rest_lower_link_pressuremat_sensor/taxels/forces", TaxelArray, self.legs_lower_pressure_collection_callback)
        plt.ion()
        plt.show()

    def head_pressure_collection_callback(self, data):
        ''' Accepts incoming coordinates from the gazebo model, and stores it into a variable local to our namespace'''
        centers_y = np.asarray(data.centers_y)
        #The x-centers will arrive in the coordinate frame specified by gazebo. We need to change that to out pressure mat frame
        self.head_centers_y = centers_y + self.bed_half_width*np.ones(centers_y.shape)
        self.head_centers_x = np.asarray(data.centers_x)

    def body_pressure_collection_callback(self, data):
        ''' Accepts incoming coordinates from the gazebo model, and stores it into a variable local to our namespace'''
        centers_y = np.asarray(data.centers_y)
        #The x-centers will arrive in the coordinate frame specified by gazebo. We need to change that to out pressure mat frame
        self.body_centers_y = centers_y + self.bed_half_width*np.ones(centers_y.shape)
        self.body_centers_x = np.asarray(data.centers_x)


    def legs_upper_pressure_collection_callback(self, data):
        ''' Accepts incoming coordinates from the gazebo model, and stores it into a variable local to our namespace'''
        centers_y = np.asarray(data.centers_y)
        #The x-centers will arrive in the coordinate frame specified by gazebo. We need to change that to out pressure mat frame
        self.legs_upper_centers_y = centers_y + self.bed_half_width*np.ones(centers_y.shape)
        self.legs_upper_centers_x = np.asarray(data.centers_x)


    def legs_lower_pressure_collection_callback(self, data):
        ''' Accepts incoming coordinates from the gazebo model, and stores it into a variable local to our namespace'''
        centers_y = np.asarray(data.centers_y)
        #The x-centers will arrive in the coordinate frame specified by gazebo. We need to change that to out pressure mat frame
        self.legs_lower_centers_y = centers_y + self.bed_half_width*np.ones(centers_y.shape)
        self.legs_lower_centers_x = np.asarray(data.centers_x)



    def distances_to_ratios(self):
        '''Converts the distances of pressure values to ratios.
        Namely, width_ratio = +/- self.center_x/(bed_width/2)
                height_ratio =  self.center_y/(bed_hight)'''

        self.height_ratio = np.concatenate((np.concatenate((self.head_centers_x/self.bed_height, self.body_centers_x/self.bed_height)), np.concatenate((self.legs_upper_centers_x/self.bed_height, self.legs_lower_centers_x/self.bed_height))))
        self.width_ratio = np.concatenate((np.concatenate((self.head_centers_y/self.bed_width, self.body_centers_y/self.bed_width)), np.concatenate((self.legs_upper_centers_y/self.bed_width, self.legs_lower_centers_y/self.bed_width))))

    
    def get_pressure_map(self):
        '''Take the pressure ratios, dimensions of the pressure map in terms of taxels x taxels,
        and multiply the ratios with the corresponding dimension, and then set the bits in a boolean matrix corresponding to the dimension obtained'''
        #Zero out the initial local matrix
        pressure_map_matrix = np.zeros([self.numoftaxels_x, self.numoftaxels_y])
        #Multiply the height ratio and width ratio pressure values by the number of taxels in the width and height of the bed
        #this will convert the ratios, which are more general, to the specific case of the vista medical pressure mat
        taxels_x = self.height_ratio*self.numoftaxels_x
        taxels_y = self.width_ratio*self.numoftaxels_y

        #Typecast into int, so that we can highlight the right taxel in the pressure matrix
        taxels_x = taxels_x.astype(int)
        taxels_y = taxels_y.astype(int)
        #Now that we have the exact pixels, we can go ahead and set selected pixels to one
        for i in range(taxels_y.shape[0]-1):
            pressure_map_matrix[taxels_x[i], taxels_y[i]] = 1
        #Assign to a global 
        self.pressure_map = pressure_map_matrix

    def visualize_pressure_map(self):
        '''Used to visualize the pressure map created'''
        plt.imshow(self.pressure_map, interpolation='nearest', cmap= plt.cm.binary, origin='upper', vmin=0, vmax=1)
        plt.draw() 
        
    def run(self):
        rate = rospy.Rate(5) #5 Hz

        while not rospy.is_shutdown():
            #Pass the centers to the conversion function so that the centers can be converted to percentages of bed width and height 
            self.distances_to_ratios()
            #Use the present percentages to get the exact pixels to mark as black
            self.get_pressure_map()
            #Call a function that displays the image matrix
            self.visualize_pressure_map()
            rate.sleep()

'''Runs the Autobed robot using an object of the AutobedClient class and the run method provided therein'''
if __name__ == "__main__":
    #Initialize pose trainer 
    rospy.init_node('pose_estimation_trainer', anonymous = True)
    pose_estimator_trainer = PoseTrainer()
    pose_estimator_trainer.run()

