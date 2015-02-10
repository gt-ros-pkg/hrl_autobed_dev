#!/usr/bin/env python
import sys
import numpy as np
import matplotlib.pyplot as plt

import roslib; roslib.load_manifest('autobed_pose_estimator')
import rospy

from hrl_haptic_manipulation_in_clutter_msgs.msg import TaxelArray 

class MapGenerator():
    '''Subscribes to the gazebo autobed and displays a 
    map of whatever object is placed on the surface 
    of the bed'''
    def __init__(self):
        '''Node that listens into the Gazebo plugin output data array, 
        which consists of coordinates of the pressure points.
        These coordinates are converted into percentages of the total 
        width/height of the bed. Once this is done, then the pressure points can be used 
        to correlate with a mat of any size and resolution. 
        Once this is done, visualization will be done.'''
        self.centers_x = np.array([])
        self.centers_y = np.array([])
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
        self.BED_WIDTH = 0.915 #metres
        self.BED_HALF_WIDTH = self.BED_WIDTH/2
        self.BED_HEIGHT = 2.08 #metres
        self.MAT_WIDTH = 0.762 #metres
        self.MAT_HEIGHT = 1.854 #metres
        self.MAT_HALF_WIDTH = self.MAT_WIDTH/2 
        self.NUMOFTAXELS_X = 64#73 #taxels
        self.NUMOFTAXELS_Y = 27#30 
        self.LOW_TAXEL_THRESH_X = 0
        self.LOW_TAXEL_THRESH_Y = 0
        self.HIGH_TAXEL_THRESH_X = (self.NUMOFTAXELS_X - 1) 
        self.HIGH_TAXEL_THRESH_Y = (self.NUMOFTAXELS_Y - 1) 
        self.PLUGIN_TO_AUTOBED_X = 0.0
        self.PLUGIN_TO_AUTOBED_Y = -self.BED_HALF_WIDTH
        #TODO: Remove this. Move Autobed to the world origin.
        self.WORLD_TO_AUTOBED_X = 1.07
        self.WORLD_TO_AUTOBED_Y = -self.BED_HALF_WIDTH
        self.AUTOBED_TO_PRESSURE_MAT_X = self.BED_HEIGHT - self.MAT_HEIGHT
        self.AUTOBED_TO_PRESSURE_MAT_Y = self.BED_HALF_WIDTH - self.MAT_HALF_WIDTH

        #Init a ROS node
        rospy.init_node('pressure_mat', anonymous = True)
        #Start subscribers to accept the incoming pressure coordinates 
        rospy.Subscriber("/head_rest_link_pressuremat_sensor/taxels/forces", TaxelArray, self.head_pressure_collection_callback)
        rospy.Subscriber("/mid_body_link_pressuremat_sensor/taxels/forces", TaxelArray, self.body_pressure_collection_callback)
        rospy.Subscriber("/leg_rest_upper_link_pressuremat_sensor/taxels/forces", TaxelArray, self.legs_upper_pressure_collection_callback)
        rospy.Subscriber("/leg_rest_lower_link_pressuremat_sensor/taxels/forces", TaxelArray, self.legs_lower_pressure_collection_callback)
        plt.ion()
        plt.show()


    def head_pressure_collection_callback(self, data):
        ''' Accepts incoming coordinates from the gazebo model, 
        and stores it into a variable local to our namespace'''
        self.head_centers_y = np.asarray(data.centers_y)
        self.head_centers_x = np.asarray(data.centers_x)


    def body_pressure_collection_callback(self, data):
        ''' Accepts incoming coordinates from the gazebo model, 
        and stores it into a variable local to our namespace'''
        self.body_centers_y = np.asarray(data.centers_y)
        self.body_centers_x = np.asarray(data.centers_x)


    def legs_upper_pressure_collection_callback(self, data):
        ''' Accepts incoming coordinates from the gazebo model,
        and stores it into a variable local to our namespace'''
        self.legs_upper_centers_y = np.asarray(data.centers_y)
        self.legs_upper_centers_x = np.asarray(data.centers_x)


    def legs_lower_pressure_collection_callback(self, data):
        ''' Accepts incoming coordinates from the gazebo model, 
        and stores it into a variable local to our namespace'''
        self.legs_lower_centers_y = np.asarray(data.centers_y)
        self.legs_lower_centers_x = np.asarray(data.centers_x)

    
    def get_mat_size_in_taxels(self):
        '''Returns the mat size in taxels x taxels'''
        return [self.NUMOFTAXELS_X, self.NUMOFTAXELS_Y]

    
    def tuple_coordinates_to_taxel_positions(self, center_tuple):
        '''Takes coordinates in the world frame and converts 
        the same to taxel positions on the mat.
        Input: Coordinates as a tuple (x,y)
        Output: Taxel positions as tuple (t_x, t_y)'''
        #Assign to local variables after a coordinate transform
        center_x = center_tuple[0] - self.WORLD_TO_AUTOBED_X - self.AUTOBED_TO_PRESSURE_MAT_X
        center_y = center_tuple[1] - self.WORLD_TO_AUTOBED_Y - self.AUTOBED_TO_PRESSURE_MAT_Y

        t_x = ((center_x/self.BED_HEIGHT)*self.NUMOFTAXELS_X).astype(int) - 1
        t_y = ((center_y/self.BED_WIDTH)*self.NUMOFTAXELS_Y).astype(int) - 1

        #thresholding(TODO: Remove this part and consider the case where body may lie partly outside)
        t_x = self.HIGH_TAXEL_THRESH_X if t_x > self.HIGH_TAXEL_THRESH_X else t_x
        t_x = self.LOW_TAXEL_THRESH_X if t_x < self.LOW_TAXEL_THRESH_X else t_x
        t_y = self.HIGH_TAXEL_THRESH_Y if t_y > self.HIGH_TAXEL_THRESH_Y else t_y
        t_y = self.LOW_TAXEL_THRESH_Y if t_y < self.LOW_TAXEL_THRESH_Y else t_y

        return (t_x, t_y)


    def tuple_taxel_positions_to_coordinates(self, center_tuple):
        '''Takes coordinates in the taxel frame and converts 
        them into coordinates in the world frame
        Input: Taxel positions as a tuple (t_x, t_y)
        Output: Coordinates as a tuple (x, y)'''
        t_x = center_tuple[0]
        t_y = center_tuple[1]
        c_x = ((t_x + 1.0)*self.BED_HEIGHT)/self.NUMOFTAXELS_X + self.AUTOBED_TO_PRESSURE_MAT_X + self.WORLD_TO_AUTOBED_X 
        c_y = ((t_y + 1.0)*self.BED_WIDTH)/self.NUMOFTAXELS_Y + self.AUTOBED_TO_PRESSURE_MAT_Y + self.WORLD_TO_AUTOBED_Y
        return (c_x + self.WORLD_TO_AUTOBED_X, c_y + self.WORLD_TO_AUTOBED_Y)


    def pool_all_centers(self):
        '''Input coordinates come from 4 different sensor locations 
        (head, torso, legs_upper and legs_lower). This function pools 
        all the centers making them one single list of centers'''
        self.centers_x = np.concatenate((np.concatenate((self.head_centers_x, self.body_centers_x)), np.concatenate((self.legs_upper_centers_x, self.legs_lower_centers_x))))
        self.centers_y = np.concatenate((np.concatenate((self.head_centers_y, self.body_centers_y)), np.concatenate((self.legs_upper_centers_y, self.legs_lower_centers_y))))

    
    def get_pressure_map(self):
        '''Pool all centers obtained from the 4 different pressure maps,
        transform these centers from world coordinates to pressure mat 
        coordinates, and then plug them into a pressure map matrix.'''
        pressure_map_matrix = np.zeros([self.NUMOFTAXELS_X, self.NUMOFTAXELS_Y])
        '''First, pool data coming from 4 different pressure sensors'''
        self.pool_all_centers() 
        '''Converts Gazebo 'pressuremat plugin' coordinates to bed coordinates'''
        bed_centers_x = self.centers_x - self.PLUGIN_TO_AUTOBED_X*np.ones(self.centers_x.shape)   
        bed_centers_y = self.centers_y - self.PLUGIN_TO_AUTOBED_Y*np.ones(self.centers_y.shape)
        '''Converts bed coordinates to mat coordinates'''
        mat_centers_x = bed_centers_x - self.AUTOBED_TO_PRESSURE_MAT_X*np.ones(bed_centers_x.shape)
        mat_centers_y = bed_centers_y - self.AUTOBED_TO_PRESSURE_MAT_Y*np.ones(bed_centers_y.shape)
        '''Convert mat coordinates to taxel position on pressure map'''
        '''Multiply the height ratio and width ratio pressure values 
        by the number of taxels in the width and height of the bed
        this will convert the ratios, which are more general, to the 
        specific case of the vista medical pressure mat'''
        taxels_x = (mat_centers_x/ self.MAT_HEIGHT)*self.NUMOFTAXELS_X
        taxels_y = (mat_centers_y/ self.MAT_WIDTH)*self.NUMOFTAXELS_Y
        '''Typecast into int, so that we can highlight the right taxel 
        in the pressure matrix, and threshold the resulting values'''
        taxels_x = (taxels_x.astype(int) - 1)
        taxels_y = (taxels_y.astype(int) - 1)
        #Thresholding the taxels_* array
        taxels_x[taxels_x < self.LOW_TAXEL_THRESH_X] = self.LOW_TAXEL_THRESH_X 
        taxels_y[taxels_y < self.LOW_TAXEL_THRESH_Y] = self.LOW_TAXEL_THRESH_Y
        taxels_x[taxels_x > self.HIGH_TAXEL_THRESH_X] = self.HIGH_TAXEL_THRESH_X
        taxels_y[taxels_y > self.HIGH_TAXEL_THRESH_Y] = self.HIGH_TAXEL_THRESH_Y 
        #Now that we have the exact pixels, we can go ahead and set selected pixels to one
        for i in range(taxels_y.shape[0]-1):
            pressure_map_matrix[taxels_x[i], taxels_y[i]] = 1
        #Assign to a global 
        #self.pressure_map =  pressure_map_matrix
        return pressure_map_matrix 


    def interpolate_pressure_map(self, pressure_map_matrix):
        '''Used to interpolate a pressure map matrix supplied as an argument'''
        for i in range(1, self.NUMOFTAXELS_X):
            for j in range(1, self.NUMOFTAXELS_Y):
                    window = pressure_map_matrix[i-1:i+2, j-1:j+2]
                    if window.sum() >= 5:
                        pressure_map_matrix[i, j] = 1
                       
        return pressure_map_matrix


    def visualize_pressure_map(self, pressure_map_matrix):
        '''Used to visualize the pressure map created'''
        plt.imshow(pressure_map_matrix, interpolation='nearest', cmap= plt.cm.binary, origin='upper', vmin=0, vmax=1)
        plt.draw() 
        
    def run(self):
        rate = rospy.Rate(5) #5 Hz

        while not rospy.is_shutdown():
            #Use the present percentages to get the exact pixels to mark as black
            pressure_map_matrix = self.get_pressure_map()
            #Call a function that displays the image matrix
            self.visualize_pressure_map(pressure_map_matrix)
            rate.sleep()




'''Runs the pressure mat using an object of the MapGenerator class 
and the run method provided therein'''
if __name__ == "__main__":
    #Initialize pressure mat
    pressure_mat = MapGenerator()
    pressure_mat.run()

