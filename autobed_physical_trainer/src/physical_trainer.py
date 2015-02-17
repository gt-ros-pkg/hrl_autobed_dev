#!/usr/bin/env python
import sys
import os
import numpy as np
import matplotlib.pyplot as plt

import cPickle as pkl

from scipy import ndimage
from skimage.feature import hog
from skimage import data, color, exposure

from sklearn.cluster import KMeans
from sklearn.decomposition import PCA

class PhysicalTrainer():
    '''Gets the dictionary of pressure maps from the training database, 
    and will have API to do all sorts of training with it.'''

    def __init__(self, training_database_file):
        '''Opens the specified pickle files to get the combined dataset:
        This dataset is a dictionary of pressure maps with the corresponding
        3d position and orientation of the markers associated with it.'''
        #Entire pressure dataset with coordinates in world frame
        dat = pkl.load(open(training_database_file, 
            "rb")) 
        #Pop the mat coordinates from the dataset
        self.world_to_mat_flattened = dat.pop('mat_o') 
        #TODO:Write code for the dataset to store these vals
        self.mat_size = (64, 27)

        self.physical_pressure_dataset = dat.keys()#Pressure maps
        self.world_frame_joints = dat.values()#Coordinates in world frame
        self.mat_frame_joints = []
       
    def bench_k_means(self, estimator, name, data):
        estimator.fit(data)
        print('% 9s  %i   %.3f   %.3f   %.3f   %.3f   %.3f %.3f'% (
            name, estimator.inertia_,
            metrics.homogeneity_score(labels,
            estimator.labels_),
            metrics.completeness_score(labels,
            estimator.labels_),
            metrics.v_measure_score(labels,
            estimator.labels_),
            metrics.adjusted_rand_score(labels,
            estimator.labels_),
            metrics.adjusted_mutual_info_score(labels,
            estimator.labels_),
            metrics.silhouette_score(data,
            estimator.labels_,
            metric='euclidean',
            sample_size=sample_size)))

    def train_hog_svm(self):
        '''Runs training on the dataset using the Upsample+ HoG + K-Means + SVM
        + Linear Regression technique'''
        pressure_map_highres = []
        flat_hog = []

        for map_index in range(len(self.physical_pressure_dataset)):
            print "Iteration number: {}".format(map_index)
            #Resize mat to make into a matrix
            pressure_map_lowres = np.resize(
                    self.physical_pressure_dataset[map_index], self.mat_size)
            #Upsample the current map using bilinear interpolation
            pressure_map_highres.append(ndimage.zoom(
                    pressure_map_lowres, 2, order=1))
            #Compute HoG of the current pressure map
            fd, hog_image = hog(
                    pressure_map_highres[map_index], orientations=8, 
                    pixels_per_cell=(4,4), cells_per_block = (1, 1), 
                    visualise=True)
            flat_hog.append(fd) 
        #Now that we have a database of HoGs , we can perform K-Means on it
        self.bench_k_means(KMeans(init='k-means++', n_clusters=n_digits, 
            n_init=10), name="k-means++", data=flat_hog)


if __name__ == "__main__":
    #Initialize trainer with a training database file
    training_database_file = sys.argv[1] #Where is the training database is 
    training_type = sys.argv[2] #Type of algorithm you want to train with
    p = PhysicalTrainer(training_database_file) 
    if training_type == 'HoG_SVM':
        p.train_hog_svm()
    else:
        'Please specify correct training type:1. HoG_SVM'

