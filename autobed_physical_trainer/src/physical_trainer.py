#!/usr/bin/env python
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import cPickle as pkl
import random
from scipy import ndimage
from skimage.feature import hog
from skimage import data, color, exposure

from sklearn.cluster import KMeans
from sklearn.preprocessing import scale
from sklearn import linear_model, decomposition, kernel_ridge, neighbors
from sklearn import metrics, cross_validation

MAT_WIDTH = 0.762 #metres
MAT_HEIGHT = 1.854 #metres
MAT_HALF_WIDTH = MAT_WIDTH/2 
NUMOFTAXELS_X = 64#73 #taxels
NUMOFTAXELS_Y = 27#30 
LOW_TAXEL_THRESH_X = 0
LOW_TAXEL_THRESH_Y = 0
HIGH_TAXEL_THRESH_X = (NUMOFTAXELS_X - 1) 
HIGH_TAXEL_THRESH_Y = (NUMOFTAXELS_Y - 1) 

 
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
        try:
            self.p_world_mat = dat.pop('mat_o') 
        except:
            print "[Warning] MAT ORIGIN HAS NOT BEEN CAPTURED."
            print "[Warning] Either retrain system or get mat older mat_origin"
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
        rand_keys = dat.keys()
        random.shuffle(rand_keys)
        self.train_y = [] #Initialize the training coordinate list
        self.test_y = [] #Initialize the ground truth list
        self.dataset_y = [] #Initialization for the entire dataset 
        self.train_x_flat = rand_keys[:6000]#Pressure maps
        [self.train_y.append(dat[key]) for key in self.train_x_flat]#Coordinates 
        self.test_x_flat = rand_keys[6000:6700]#Pressure maps(test dataset)
        [self.test_y.append(dat[key]) for key in self.test_x_flat]#ground truth
        self.dataset_x_flat = rand_keys[:6000]#Pressure maps
        [self.dataset_y.append(dat[key]) for key in self.dataset_x_flat]
        self.cv_fold = 5 # Value of k in k-fold cross validation 
        self.mat_frame_joints = []


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
 

    def train_hog_linear(self):
        '''Runs training on the dataset using the Upsample+ HoG 
        + Linear Regression technique'''
        
        #Resize incoming pressure map
        pressure_map_dataset_lowres_train = (
                self.preprocessing_pressure_array_resize(self.dataset_x_flat))
        #Upsample the lowres training dataset 
        pressure_map_dataset_highres_train = (
            self.preprocessing_pressure_map_upsample(
                pressure_map_dataset_lowres_train))
        #Compute HoG of the current(training) pressure map dataset
        pressure_hog_train = self.compute_HoG(
                pressure_map_dataset_highres_train)

        #OPTIONAL: PCA STAGE
        #X = self.pca_pressure_map( self.train_y, False)
        #Now we train a linear classifier on the dataset of HoGs

        self.regr = linear_model.LinearRegression()
        scores = cross_validation.cross_val_score(
            self.regr, pressure_hog_train, self.dataset_y, cv=self.cv_fold)
        print("Accuracy after k-fold cross validation: %0.2f (+/- %0.2f)" 
                % (scores.mean(), scores.std() * 2))

        #predicted = cross_validation.cross_val_predict(
                #self.regr, np.asarray(pressure_hog_train), 
                #np.asarray(self.dataset_y), cv=self.cv_fold)
        ##mean squared error
        #print("residual sum of squares: %.8f"
              #% np.mean((predicted - self.dataset_y) **2))

        # Train the model using the training sets
        self.regr.fit(pressure_hog_train, self.dataset_y)
        #Pickle the trained model
        pkl.dump(self.regr, open('./dataset/trained_model_'+sys.argv[2]+'.p'
                ,'wb'))
    

    def train_hog_ridge(self):
        '''Runs training on the dataset using the Upsample+ HoG+
        + Ridge Regression technique'''
        
        #Resize incoming pressure map
        pressure_map_dataset_lowres_train = (
                self.preprocessing_pressure_array_resize(self.dataset_x_flat))
        #Upsample the lowres training dataset 
        pressure_map_dataset_highres_train = (
            self.preprocessing_pressure_map_upsample(
                pressure_map_dataset_lowres_train))
        #Compute HoG of the current(training) pressure map dataset
        pressure_hog_train = self.compute_HoG(
                pressure_map_dataset_highres_train)

        #OPTIONAL: PCA STAGE
        #X = self.pca_pressure_map( self.train_y, False)
        #Now we train a Ridge regression on the dataset of HoGs
        self.regr = linear_model.Ridge(alpha=1.0)
        scores = cross_validation.cross_val_score(
            self.regr, pressure_hog_train, self.dataset_y, cv=self.cv_fold)
        print("Accuracy after k-fold cross validation: %0.2f (+/- %0.2f)" 
                % (scores.mean(), scores.std() * 2))
        #predicted = cross_validation.cross_val_predict(
                #self.regr, pressure_hog_train, self.dataset_y, cv=self.cv_fold)
        ##Mean Squared Error
        #print("Residual sum of squares: %.8f"
              #% np.mean((predicted - self.dataset_y) **2))
 
        # Train the model using the training sets
        self.regr.fit(pressure_hog_train, self.dataset_y)
        #Pickle the trained model
        pkl.dump(self.regr, open('./dataset/trained_model_'
            +sys.argv[2]+'.p', 'wb'))
 

    def train_hog_krr(self):
        '''Runs training on the dataset using the Upsample+ HoG+
        + Kernel Ridge Regression technique'''
        
        #Resize incoming pressure map
        pressure_map_dataset_lowres_train = (
                self.preprocessing_pressure_array_resize(self.dataset_x_flat))
        #Upsample the lowres training dataset 
        pressure_map_dataset_highres_train = (
            self.preprocessing_pressure_map_upsample(
                pressure_map_dataset_lowres_train))
        #Compute HoG of the current(training) pressure map dataset
        pressure_hog_train = self.compute_HoG(
                pressure_map_dataset_highres_train)
        #OPTIONAL: PCA STAGE
        #X = self.pca_pressure_map( self.train_y, False)
        #Now we train a Ridge regression on the dataset of HoGs
        self.regr = kernel_ridge.KernelRidge(alpha=1, kernel='rbf', gamma =
                10)
        scores = cross_validation.cross_val_score(
            self.regr, pressure_hog_train, self.dataset_y, cv=self.cv_fold)
        print("Accuracy after k-fold cross validation: %0.2f (+/- %0.2f)" 
                % (scores.mean(), scores.std() * 2))
        #predicted = cross_validation.cross_val_predict(
                #self.regr, pressure_hog_train, self.dataset_y, cv=self.cv_fold)
        ##Mean Squared Error
        #print("Residual sum of squares: %.8f"
              #% np.mean((predicted - self.dataset_y) **2))
 
        # Train the model using the training sets
        self.regr.fit(pressure_hog_train, self.dataset_y)
        #Pickle the trained model
        pkl.dump(self.regr, open('./dataset/trained_model_'
            +sys.argv[2]+'.p', 'wb'))
 

    def train_hog_knn(self):
        '''Runs training on the dataset using the Upsample+ HoG+
        + K Nearest Neighbor Regression technique'''
        #Number of neighbors
        n_neighbors = 5 
        #Resize incoming pressure map
        pressure_map_dataset_lowres_train = (
                self.preprocessing_pressure_array_resize(self.dataset_x_flat))
        #Upsample the lowres training dataset 
        pressure_map_dataset_highres_train = (
            self.preprocessing_pressure_map_upsample(
                pressure_map_dataset_lowres_train))
        #Compute HoG of the current(training) pressure map dataset
        pressure_hog_train = self.compute_HoG(
                pressure_map_dataset_highres_train)

        #OPTIONAL: PCA STAGE
        #X = self.pca_pressure_map( self.train_y, False)
        #Now we train a Ridge regression on the dataset of HoGs
        self.regr = neighbors.KNeighborsRegressor(n_neighbors,
        weights='distance')

        scores = cross_validation.cross_val_score(
            self.regr, pressure_hog_train, self.dataset_y, cv=self.cv_fold)
        print("Accuracy after k-fold cross validation: %0.2f (+/- %0.2f)" 
                % (scores.mean(), scores.std() * 2))

        #predicted = cross_validation.cross_val_predict(
                #self.regr, pressure_hog_train, self.dataset_y, cv=self.cv_fold)
        ##Mean Squared Error
        #print("Residual sum of squares: %.8f"
              #% np.mean((predicted - self.dataset_y) **2))
 
        
        # Train the model using the training sets
        self.regr.fit(pressure_hog_train, self.dataset_y)
        #Pickle the trained model
        pkl.dump(self.regr, open('./dataset/trained_model_'
            +sys.argv[2]+'.p', 'wb'))
 

    def train_hog_KMeans_SVM_Linear(self):
        '''Runs training on the dataset using the Upsample+ HoG 
        + KMeans + SVM + Ridge Regression technique'''
        n_clusters = 3 #Number of KMeans Clusters 
        #Resize incoming pressure map
        pressure_map_dataset_lowres_train = (
                self.preprocessing_pressure_array_resize(self.dataset_x_flat))
        #Upsample the lowres training dataset 
        pressure_map_dataset_highres_train = (
            self.preprocessing_pressure_map_upsample(
                pressure_map_dataset_lowres_train))
        #Compute HoG of the current(training) pressure map dataset
        pressure_hog_train = self.compute_HoG(
                pressure_map_dataset_highres_train)

        k_means = cluster.KMeans(n_clusters=n_clusters, n_init=4)
        k_means.fit(pressure_hog_train)
        labels = k_means.labels_
        svm_classifier = svm.SVC()
        svm_classifier.fit(pressure_hog_train, labels)
        #OPTIONAL: PCA STAGE
        #X = self.pca_pressure_map( self.train_y, False)
        #Now we train a linear classifier on the dataset of HoGs
        self.regr = linear_model.LinearRegression()
        scores = cross_validation.cross_val_score(
            self.regr, pressure_hog_train, self.dataset_y, cv=5)
        print("Accuracy after k-fold cross validation: %0.2f (+/- %0.2f)" 
                % (scores.mean(), scores.std() * 2))
        # Train the model using the training sets
        self.regr.fit(pressure_hog_train, self.dataset_y)
        #Pickle the trained model
        pkl.dump(self.regr, open('./dataset/trained_model_'+sys.argv[2]+'.p'
                ,'wb'))
    


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
        try:
            print('Coefficients: \n', self.regr.coef_)
        except AttributeError:
            pass
        # The mean square error
        print("Residual sum of squares: %.8f"
              % np.mean((self.regr.predict(test_hog) - self.test_y) **2))
        # Explained variance score: 1 is perfect prediction
        print('Variance score: %.8f' % self.regr.score(test_hog, self.test_y))

        #Plot n test poses at random

        estimated_y = self.regr.predict(test_hog)

        #plt.subplot(131)
        #taxel_est = []
        #taxel_real = []
        #img = random.randint(1, len(test_x_lowres)-1)
        #[taxel_est.append(self.world_to_mat(item)) for item in (
            #list(self.chunks(estimated_y[img], 3)))]
        #for item in taxel_est:
            #test_x_lowres[img][item[0], item[1]] = 200
        #print taxel_est
        #[taxel_real.append(self.world_to_mat(item)) for item in (
            #list(self.chunks(self.test_y[img], 3)))]
        #for item in taxel_real:
            #test_x_lowres[img][item[0], item[1]] = 300
        #print taxel_real
        #self.visualize_pressure_map(test_x_lowres[img])
        
        #plt.subplot(132)
        #taxel_est = []
        #taxel_real = []
        #img = random.randint(1, len(test_x_lowres)-1)
        #[taxel_est.append(self.world_to_mat(item)) for item in (
            #list(self.chunks(estimated_y[img], 3)))]
        #for item in taxel_est:
            #test_x_lowres[img][item[0], item[1]] = 200
        #print taxel_est
        #[taxel_real.append(self.world_to_mat(item)) for item in (
            #list(self.chunks(self.test_y[img], 3)))]
        #for item in taxel_real:
            #test_x_lowres[img][item[0], item[1]] = 300
        #print taxel_real 
        #self.visualize_pressure_map(test_x_lowres[img])

        #plt.subplot(133)
        #taxel_est = []
        #taxel_real = []
        #img = random.randint(1, len(test_x_lowres)-1)
        #[taxel_est.append(self.world_to_mat(item)) for item in (
            #list(self.chunks(estimated_y[img], 3)))]
        #for item in taxel_est:
            #test_x_lowres[img][item[0], item[1]] = 200
        #print taxel_est
        #[taxel_real.append(self.world_to_mat(item)) for item in (
            #list(self.chunks(self.test_y[img], 3)))]
        #for item in taxel_real:
            #test_x_lowres[img][item[0], item[1]] = 300
        #print taxel_real 
        #self.visualize_pressure_map(test_x_lowres[img])
        #plt.show()


    def chunks(self, l, n):
        """ Yield successive n-sized chunks from l.
        """
        for i in xrange(0, len(l), n):
            yield l[i:i+n]


    def pca_pressure_map(self, data, visualize = True):
        '''Computing the 3D PCA of the dataset given to it. If visualize is set
        to True, we can also visualize the output of this function'''
        X = data
        if visualize:
            fig = plt.figure(1, figsize=(4, 3))
            plt.clf()
            ax = Axes3D(fig, rect=[0, 0, .95, 1], elev=48, azim=134)
            plt.cla()

        pca = decomposition.PCA(n_components=3)
        pca.fit(X)
        X = pca.transform(X)
        if visualize:
            ax.scatter(X[:, 0], X[:, 1], X[:, 2], cmap=plt.cm.spectral)
            x_surf = [X[:, 0].min(), X[:, 0].max(),
                              X[:, 0].min(), X[:, 0].max()]
            y_surf = [X[:, 0].max(), X[:, 0].max(),
                              X[:, 0].min(), X[:, 0].min()]
            x_surf = np.array(x_surf)
            y_surf = np.array(y_surf)
            v0 = pca.transform(pca.components_[0])
            v0 /= v0[-1]
            v1 = pca.transform(pca.components_[1])
            v1 /= v1[-1]

            ax.w_xaxis.set_ticklabels([])
            ax.w_yaxis.set_ticklabels([])
            ax.w_zaxis.set_ticklabels([])

            plt.show()
        
        return X


    def world_to_mat(self, w_data):
        '''Converts a vector in the world frame to a vector in the map frame.
        Depends on the calibration of the MoCap room. Be sure to change this 
        when the calibration file changes. This function mainly helps in
        visualizing the joint coordinates on the pressure mat.
        Input: w_data: which is a 3 x 1 vector in the world frame'''
        #The homogenous transformation matrix from world to mat
        O_m_w = np.matrix([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
        p_mat_world = O_m_w.dot(-np.asarray(self.p_world_mat))
        B_m_w = np.concatenate((O_m_w, p_mat_world.T), axis=1)
        last_row = np.array([[0, 0, 0, 1]])
        B_m_w = np.concatenate((B_m_w, last_row), axis=0)
        w_data = np.append(w_data, np.array([1]))
        #Convert input to the mat frame vector
        m_data = B_m_w.dot(w_data)
        m_data = np.squeeze(np.asarray(m_data))
        #Convert this into taxels
        taxels_x = (m_data[0]/ MAT_HEIGHT)*NUMOFTAXELS_X
        taxels_y = (m_data[1]/ MAT_WIDTH)*NUMOFTAXELS_Y
        '''Typecast into int, so that we can highlight the right taxel 
        in the pressure matrix, and threshold the resulting values'''
        taxels_x = (taxels_x.astype(int) - 1)
        taxels_y = (taxels_y.astype(int) - 1)
        #Thresholding the taxels_* array
        taxels_x = LOW_TAXEL_THRESH_X if (taxels_x <= 
                LOW_TAXEL_THRESH_X) else taxels_x
        taxels_y = LOW_TAXEL_THRESH_Y if (taxels_y <=
                LOW_TAXEL_THRESH_Y) else taxels_y
        taxels_x = HIGH_TAXEL_THRESH_X if (taxels_x >= 
                HIGH_TAXEL_THRESH_X) else taxels_x
        taxels_y = HIGH_TAXEL_THRESH_Y if (taxels_y >
                HIGH_TAXEL_THRESH_Y) else taxels_y
            
        return [taxels_x, taxels_y]


    def visualize_pressure_map(self, pressure_map_matrix):
        '''Visualizing a plot of the pressure map'''
        plt.imshow(pressure_map_matrix, interpolation='nearest', cmap=
                plt.cm.bwr, origin='upper', vmin=0, vmax=100)
        return


if __name__ == "__main__":
    #Initialize trainer with a training database file
    training_database_file = sys.argv[1] #Where is the training database is 
    training_type = sys.argv[2] #Type of algorithm you want to train with
    p = PhysicalTrainer(training_database_file) 
    if training_type == 'HoG_Linear':
        p.train_hog_linear()
    elif training_type == 'HoG_Ridge':
        p.train_hog_ridge()
    elif training_type == 'HoG_KNN':
        p.train_hog_knn()
    elif training_type == 'HoG_KRR':
        p.train_hog_krr()
    else:
        'Please specify correct training type:1. HoG_Linear 2. HoG_Ridge'
    p.test_learning_algorithm()
