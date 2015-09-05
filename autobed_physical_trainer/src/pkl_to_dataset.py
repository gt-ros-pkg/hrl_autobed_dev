#!/usr/bin/env python
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import cPickle as pkl
import random
from scipy import ndimage
from skimage import data, color, exposure

from sklearn.decomposition import PCA

MAT_WIDTH = 0.762 #metres
MAT_HEIGHT = 1.854 #metres
MAT_HALF_WIDTH = MAT_WIDTH/2 
NUMOFTAXELS_X = 64#73 #taxels
NUMOFTAXELS_Y = 27#30 
LOW_TAXEL_THRESH_X = 0
LOW_TAXEL_THRESH_Y = 0
HIGH_TAXEL_THRESH_X = (NUMOFTAXELS_X - 1) 
HIGH_TAXEL_THRESH_Y = (NUMOFTAXELS_Y - 1) 

 
class DatabaseCreator():
    '''Gets the directory of pkl database and iteratively go through each file,
    cutting up the pressure maps and creating synthetic database'''
    def __init__(self, training_database_pkl_directory):

        home_sup_dat = pkl.load(
                open(training_database_pkl_directory'/home_sup.p', "rb")) 
       #Pop the mat coordinates from the dataset
        try:
            self.p_world_mat = home_sup_dat.pop('mat_o') 
        except:
            print "[Warning] MAT ORIGIN HAS NOT BEEN CAPTURED."
            print "[Warning] Either retrain system or get mat older mat_origin"
        self.mat_size = (NUMOFTAXELS_X, NUMOFTAXELS_Y)
        #Remove empty elements from the dataset, that may be due to Motion
        #Capture issues.
        print "Checking database for empty values."
        empty_count = 0
        for dict_entry in list(home_sup_dat.keys()):
            if len(home_sup_dat[dict_entry]) < (30) or (len(dict_entry) <
                    self.mat_size[0]*self.mat_size[1]):
                empty_count += 1
                del home_sup_dat[dict_entry]
        print "Empty value check results: {} rogue entries found".format(
                empty_count)

        home_sup_pressure_map = home_sup_dat.keys()[0]
        home_sup_joint_pos = home_sup_dat[home_sup_pressure_map] 
        ([self.R_sup, self.COM_sup, self.image_slices_sup,
            self.target_slices_sup]) = self.preprocess_home_position(
                                    home_sup_pressure_map, home_sup_joint_pos)


    def preprocess_home_position(self, p_map_flat, target):
        '''Performs PCA on binarized pressure map, rotates and translates
        pressure map by principal axis. Then rotates and translates target
        values by same amount in the frame of the mat. Finally slices the home
        position pressure map into 6 regions and returns sliced matrices'''
        #Reshape to create 2D pressure map
        p_map = np.asarray(np.reshape(p_map_flat, self.mat_size))
        #Binarize pressure map
        p_map_bin = np.asarray(p_map[:])
        p_map_bin[p_map_bin > 0] = 1  # All low values set to 0
        #Get the nonzero indices
        nzero_indices = np.nonzero(p_map)  
        #Perform PCA on the non-zero elements of the pressure map
        pca_x_tuples = zip([nzero_indices[0]*(-1) + (MAT_HEIGHT-1),
                            nzero_indices[1])
        pca_x = [list(elem) for elem in pca_x_tuples]  
        pca_y = [p_map[elem] for elem in zip(nzero_indices[0],
            nzero_indices[1])]
        pca = PCA(n_components=3)
        pca.fit(pca_x)
        rot_x = pca.transform(pca_x)
        



    def run(self):
        '''Uses the Rotation, translation, and slices obtained in
        initialization to create a synthetic database of images and ground 
        truth values'''

if __name__ == "__main__":
    #Initialize trainer with a training database file
    training_database_pkl_directory = sys.argv[1] #Where is the training database is 
    p = DatabaseCreator(training_database_pkl_directory) 
    p.run()
