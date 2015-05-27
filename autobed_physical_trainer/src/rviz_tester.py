#!/usr/bin/env python
import sys
import os
import numpy as np
import operator


import cPickle as pkl
from scipy import ndimage
from skimage.feature import hog
from skimage import data, color, exposure

from sklearn.cluster import KMeans
from sklearn import metrics
from sklearn.preprocessing import scale
from sklearn import linear_model
from sklearn import decomposition 

import roslib; roslib.load_manifest('autobed_physical_trainer')
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from hrl_msgs.msg import FloatArrayBare
from geometry_msgs.msg import TransformStamped


MAT_WIDTH = 0.762 #metres
MAT_HEIGHT = 1.854 #metres
MAT_HALF_WIDTH = MAT_WIDTH/2 
NUMOFTAXELS_X = 64#73 #taxels
NUMOFTAXELS_Y = 27#30 
LOW_TAXEL_THRESH_X = 0
LOW_TAXEL_THRESH_Y = 0
HIGH_TAXEL_THRESH_X = (NUMOFTAXELS_X - 1) 
HIGH_TAXEL_THRESH_Y = (NUMOFTAXELS_Y - 1) 



 
class Rviz_tester():
    '''Gets a testing dataset, and a trained model from the ../dataset/ folder
    and has API to predict the pose of the test dataset'''
    def __init__(self, test_database_file, trained_model):
        '''Opens the specified pickle files to get the combined dataset:
        This dataset is a dictionary of pressure maps with the corresponding
        3d position and orientation of the markers associated with it.'''
        rospy.init_node('abd_pose_est_layer', anonymous=True) 

        #Entire pressure dataset with coordinates in world frame
        dat = pkl.load(open(test_database_file, "rb")) 
        self.regr = pkl.load(open(trained_model, "rb"))
        #TODO:Write code for the dataset to store these vals
        self.mat_size = (NUMOFTAXELS_X, NUMOFTAXELS_Y)
        #Remove empty elements from the dataset, that may be due to Motion
        #Capture issues.
        print "Checking database for empty values."
        empty_count = 0
        for dict_entry in list(dat.keys()):
            if len(dat[dict_entry]) < (21) or (len(dict_entry) <
                    self.mat_size[0]*self.mat_size[1]):
                empty_count += 1
                del dat[dict_entry]
        print "Empty value check results: {} rogue entries found".format(
                empty_count)

        #Randomize the keys of the dictionary
        dat_keys = dat.keys()
        self.test_x_flat = dat_keys[:100]#Pressure maps(test dataset)
        self.test_y = [] #Initialize the ground truth list
        [self.test_y.append(dat[key]) for key in self.test_x_flat]#ground truth
        #Create RViz publisher and marker array.
        self.pub = rospy.Publisher('rviz_pose', MarkerArray)
        self.marker_array = MarkerArray()


    def compute_HoG(self, data):
        '''Computes a HoG(Histogram of gradients for a list of images provided
        to it. Returns a list of flattened lists'''
        flat_hog = []
        print "*****Begin HoGing the dataset*****"
        for index in range(len(data)):
            print "HoG being applied over image number: {}".format(index)
            #Compute HoG of the current pressure map
            fd, hog_image = hog(data[index], orientations=8, 
                    pixels_per_cell=(4,4), cells_per_block = (1, 1), 
                    visualise=True)
            flat_hog.append(fd) 
        return flat_hog


    def preprocessing_pressure_array_resize(self, data):
        '''Will resize all elements of the dataset into the dimensions of the 
        pressure map'''
        p_map_dataset = []
        for map_index in range(len(data)):
            #Resize mat to make into a matrix
            p_map = np.resize(data[map_index], self.mat_size)
            p_map_dataset.append(p_map)
        return p_map_dataset


    def preprocessing_pressure_map_upsample(self, data, multiple=2, order=1):
        '''Will upsample an incoming pressure map dataset'''
        p_map_highres_dataset = []
        for map_index in range(len(data)):
            #Upsample the current map using bilinear interpolation
            p_map_highres_dataset.append(
                    ndimage.zoom(data[map_index], multiple, order=order))
        return p_map_highres_dataset
 

    def test_learning_algorithm(self):
        '''Tests the learning algorithm we're trying to implement'''
        test_x_lowres = (
            self.preprocessing_pressure_array_resize(self.test_x_flat))
        #Upsample the current map using bilinear interpolation
        test_x_highres = self.preprocessing_pressure_map_upsample(
                test_x_lowres)
        #Compute HoG of the current(test) pressure map dataset
        test_hog = self.compute_HoG(test_x_highres)

        # The coefficients
        print('Coefficients: \n', self.regr.coef_)
        # The mean square error
        print("Residual sum of squares: %.8f"
              % np.mean((self.regr.predict(test_hog) - self.test_y) **2))
        # Explained variance score: 1 is perfect prediction
        print('Variance score: %.8f' % self.regr.score(test_hog, self.test_y))
        self.estimated_y = self.regr.predict(test_hog)


    def rviz_visualization(self):
        '''Visualizes all the markers using the information from the
        estimated_y variable. Publishes this visualization to RViz'''

        rate = rospy.Rate(1) #1 Hz
        for current_est in self.estimated_y:
            self.marker_array.markers = []
            i = 0
            marker_id = 0
            while i in range(len(current_est)):
                marker = Marker()
                marker.header.frame_id = "/world"
                marker.id = marker_id
                marker.type = marker.ARROW
                marker.action = marker.ADD
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = -1.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.pose.position.x = current_est[i]
                marker.pose.position.y = current_est[i + 1]
                marker.pose.position.z = current_est[i + 2]
                self.marker_array.markers.append(marker)
                marker_id += 1
                i += 3
            self.pub.publish(self.marker_array)
            rate.sleep()



if __name__ == "__main__":
    rviz_plotter = Rviz_tester(sys.argv[1], sys.argv[2])
    rviz_plotter.test_learning_algorithm()
    rviz_plotter.rviz_visualization()
