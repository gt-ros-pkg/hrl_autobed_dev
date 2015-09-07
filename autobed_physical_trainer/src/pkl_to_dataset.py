#!/usr/bin/env python
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import cPickle as pkl
import random
from scipy import ndimage
## from skimage import data, color, exposure

from sklearn.decomposition import PCA

MAT_WIDTH = 0.762 #metres
MAT_HEIGHT = 1.854 #metres
MAT_HALF_WIDTH = MAT_WIDTH/2 
NUMOFTAXELS_X = 64#73 #taxels
NUMOFTAXELS_Y = 27#30 
INTER_SENSOR_DISTANCE = 0.0286#metres
LOW_TAXEL_THRESH_X = 0
LOW_TAXEL_THRESH_Y = 0
HIGH_TAXEL_THRESH_X = (NUMOFTAXELS_X - 1) 
HIGH_TAXEL_THRESH_Y = (NUMOFTAXELS_Y - 1) 

 
class DatabaseCreator():
    '''Gets the directory of pkl database and iteratively go through each file,
    cutting up the pressure maps and creating synthetic database'''
    def __init__(self, training_database_pkl_directory):

        home_sup_dat = pkl.load(
                open(training_database_pkl_directory+'home_sup.p', "rb")) 
       #Pop the mat coordinates from the dataset
        try:
            self.p_world_mat = home_sup_dat.pop('mat_o') 
        except:
            print "[Warning] MAT ORIGIN HAS NOT BEEN CAPTURED."
            print "[Warning] Either retrain system or get mat older mat_origin"
            self.p_world_mat = [0.289, 1.861, 0.546]

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
        home_sup_joint_pos_world = self.preprocess_targets(
                                            home_sup_dat[home_sup_pressure_map]) 

        #Targets in the mat frame
        home_sup_joint_pos = (
                [self.world_to_mat(elem) for elem in home_sup_joint_pos_world])

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
        #Get the nonzero indices
        nzero_indices = np.nonzero(p_map)
        #Perform PCA on the non-zero elements of the pressure map
        pca_x_tuples = zip(nzero_indices[1], 
                                    nzero_indices[0]*(-1) + (NUMOFTAXELS_X-1))
        pca_x_pixels = np.asarray([list(elem) for elem in pca_x_tuples])
        pca_y_pixels = [p_map[elem] for elem in zip(nzero_indices[0],
            nzero_indices[1])]

        #Perform PCA in the space of pressure mat pixels
        self.pca_pixels = PCA(n_components=2)
        self.pca_pixels.fit(pca_x_pixels)
        #The output of PCA needs rotation by -90 
        rot_x_pixels = self.pca_pixels.transform(pca_x_pixels)
        rot_x_pixels = np.dot(np.asarray(rot_x_pixels),
                                                np.array([[0, -1],[-1, 0]]))
        rot_trans_x_pixels = np.asarray(
                [np.asarray(elem) + np.array([NUMOFTAXELS_Y/2, NUMOFTAXELS_X/2]) 
                for elem in rot_x_pixels]) 
        rot_trans_x_pixels = rot_trans_x_pixels.astype(int)
        #Thresholding the rotated matrices
        rot_trans_x_pixels[rot_trans_x_pixels < LOW_TAXEL_THRESH_X] = (
                                                 LOW_TAXEL_THRESH_X)            
        rot_trans_x_pixels[rot_trans_x_pixels[:, 1] >= NUMOFTAXELS_X] = (
                                                            NUMOFTAXELS_X - 1)

        rot_trans_x_pixels[rot_trans_x_pixels[:, 0] >= NUMOFTAXELS_Y] = (
                                                            NUMOFTAXELS_Y - 1)

        rotated_p_map_coord = ([tuple([(-1)*(elem[1] - (NUMOFTAXELS_X - 1)), 
                                    elem[0]]) for elem in rot_trans_x_pixels])
        #Creating rotated p_map
        rotated_p_map = np.zeros([NUMOFTAXELS_X, NUMOFTAXELS_Y])
        print rotated_p_map_coord
        for i in range(len(pca_y_pixels)):
            print rotated_p_map_coord[i]
            rotated_p_map[rotated_p_map_coord[i]] = pca_y_pixels[i]/2.0
        #self.visualize_pressure_map(rotated_p_map)
        #plt.show()

        #Taxels in 3D space in the mat frame
        pca_x_mat = INTER_SENSOR_DISTANCE*pca_x_pixels
        #We need only X,Y coordinates in the mat frame
        targets_mat = np.asarray([[elem[0], elem[1]] for elem in target])

        #Perform PCA in the 3D Space with the mat origin
        self.pca_mat = PCA(n_components=2)
        self.pca_mat.fit(pca_x_mat)
        #The output of PCA needs rotation by -90 
        rot_targets_mat = self.pca_mat.transform(targets_mat)
        rot_targets_mat = np.dot(np.asarray(rot_targets_mat),
                                                np.array([[0, -1],[-1, 0]]))
        
        ## print rot_targets_mat 
        rot_trans_targets_mat = np.asarray(
            [np.asarray(elem) + 
            INTER_SENSOR_DISTANCE*np.array([NUMOFTAXELS_Y/2, NUMOFTAXELS_X/2]) 
            for elem in rot_targets_mat]) 

        #Run matching function to find the best rotation offset
        ang_offset, trans_offset = self.getOffset(rot_trans_targets_mat, rotated_p_map)
        rot_trans_targets_mat = np.dot(np.asarray(rot_trans_targets_mat),
                                       np.array([[np.cos(ang_offset), -np.sin(ang_offset)],
                                                 [np.sin(ang_offset), np.cos(ang_offset)]]))
        rot_trans_targets_mat = rot_trans_targets_mat + trans_offset        
        
        # Visualzation of head and ankle part only
        ## part_map = np.zeros(np.shape(rotated_p_map))
        ## for i in xrange(len(rotated_p_map)):
        ##     for j in xrange(len(rotated_p_map[i])):
        ##         if i>1 and i<13 and rotated_p_map[i,j] > 10.0:
        ##             part_map[i,j] = 50.0
        ##         if i>len(rotated_p_map)-8 and i < len(rotated_p_map)-2 and rotated_p_map[i,j] > 10.0:
        ##             part_map[i,j] = 50.0        
        ## rotated_p_map = part_map
        
        
        ## print rot_trans_targets_mat
        rot_trans_targets_pixels = ([self.mat_to_taxels(elem) for elem in
                rot_trans_targets_mat]) 

        rotated_target_coord = ([tuple([(-1)*(elem[1] - (NUMOFTAXELS_X - 1)), 
                                    elem[0]]) for elem in rot_trans_targets_pixels])

        print rotated_target_coord
        for i in range(len(rotated_target_coord)):
            rotated_p_map[rotated_target_coord[i]] = 100
        self.visualize_pressure_map(rotated_p_map)
        plt.show()




    def preprocess_targets(self, targets):
        '''Converts the target values from a single list to a list of lists'''
        output = []
        index = 0
        while index <= (len(targets)-3):
            output.append([targets[index], targets[index+1], targets[index+2]])
            index += 3
        return output


    def world_to_mat(self, w_data):
        '''Converts a vector in the world frame to a vector in the map frame.
        Depends on the calibration of the MoCap room. Be sure to change this 
        when the calibration file changes. This function mainly helps in
        visualizing the joint coordinates on the pressure mat.
        Input: w_data: which is a 3 x 1 vector in the world frame'''
        #The homogenous transformation matrix from world to mat
        O_m_w = np.matrix([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
        p_mat_world = O_m_w.dot(-np.asarray(self.p_world_mat))
        B_m_w = np.concatenate((O_m_w, p_mat_world.T), axis=1)
        last_row = np.array([[0, 0, 0, 1]])
        B_m_w = np.concatenate((B_m_w, last_row), axis=0)
        w_data = np.append(w_data, np.array([1]))
        #Convert input to the mat frame vector
        m_data = B_m_w.dot(w_data)
        m_data = np.squeeze(np.asarray(m_data))
        return m_data


    def mat_to_taxels(self, m_data):
        #Convert coordinates in 3D space in the mat frame into taxels
        taxels_x = (m_data[1]/ INTER_SENSOR_DISTANCE)
        taxels_y = (m_data[0]/ INTER_SENSOR_DISTANCE)
        '''Typecast into int, so that we can highlight the right taxel 
        in the pressure matrix, and threshold the resulting values'''
        taxels_x = (taxels_x.astype(int) )
        taxels_y = (taxels_y.astype(int) )
        #Thresholding the taxels_* array
        taxels_x = LOW_TAXEL_THRESH_X if (taxels_x <= 
                LOW_TAXEL_THRESH_X) else taxels_x
        taxels_y = LOW_TAXEL_THRESH_Y if (taxels_y <=
                LOW_TAXEL_THRESH_Y) else taxels_y
        taxels_x = HIGH_TAXEL_THRESH_X if (taxels_x >= 
                HIGH_TAXEL_THRESH_X) else taxels_x
        taxels_y = HIGH_TAXEL_THRESH_Y if (taxels_y >
                HIGH_TAXEL_THRESH_Y) else taxels_y
        return [taxels_y, taxels_x]


    def visualize_pressure_map(self, pressure_map_matrix):
        '''Visualizing a plot of the pressure map'''
        plt.imshow(pressure_map_matrix, interpolation='nearest', cmap=
                plt.cm.bwr, origin='upper', vmin=0, vmax=100)
        return


    def getOffset(self, target_mat, p_map):
        '''Find the best angular and translation offset''' 

        iteration   = 500
        ang_offset  = 0.0
        ang_range   = [0.0, 10.0/180.0*np.pi]
        x_range     = [0.0, 0.15]
        y_range     = [0.0, 0.15]
        max_score   = 0.
        min_variance = 1000.0
        best_offset = np.array([0.,0.,0.]) #ang, x, y
        window_size = 2

        map_pressure_thres = 10.0
        head_pixel_range  = [2,12]
        ankle_pixel_range = [-8,-2]
        
        # get head and food parts in map
        part_map = np.zeros(np.shape(p_map))
        for i in xrange(len(p_map)):
            for j in xrange(len(p_map[i])):
                if i>=head_pixel_range[0] and i<=head_pixel_range[1] and p_map[i,j] > map_pressure_thres:
                    part_map[i,j] = 50.0
                if i>len(p_map)+ankle_pixel_range[0] and i < len(p_map)+ankle_pixel_range[1] \
                  and p_map[i,j] > map_pressure_thres:
                    part_map[i,j] = 50.0        
        ## part_map[0:13,:] = p_map[0:13,:]
        ## part_map[-5:-1,:] = p_map[-5:-1,:]
        p_map = part_map        

        # Q: p_map is not normalized and scale is really different
        while iteration>0:
            iteration = iteration - 1

            # random angle
            ang = random.uniform(ang_range[0], ang_range[1])
            x   = random.uniform(x_range[0], x_range[1])
            y   = random.uniform(y_range[0], y_range[1])

            # rotate target mat
            rot_trans_targets_mat = np.dot(np.asarray(target_mat),
                                           np.array([[np.cos(ang), -np.sin(ang)],
                                                     [np.sin(ang), np.cos(ang)]]))        
            ## print np.shape(rot_trans_targets_mat), rot_trans_targets_mat[0]
            rot_trans_targets_mat = rot_trans_targets_mat + np.array([x,y])

            rot_trans_targets_pixels = ([self.mat_to_taxels(elem) for elem in
                    rot_trans_targets_mat]) 

            rotated_target_coord = ([tuple([(-1)*(elem[1] - (NUMOFTAXELS_X - 1)), 
                                        elem[0]]) for elem in rot_trans_targets_pixels])
            
            # sum of scores
            score = self.pressure_score_in_window(p_map, rotated_target_coord[0], window_size) +\
              self.pressure_score_in_window(p_map, rotated_target_coord[-2], window_size) +\
              self.pressure_score_in_window(p_map, rotated_target_coord[-1], window_size)
              
            ## print iteration, " : ", score
            if score[0] > max_score or (score[0] == max_score and score[1] < min_variance):
                max_score    = score[0]
                min_variance = score[1]
                best_offset[0] = ang
                best_offset[1] = x
                best_offset[2] = y

        print "Best offset (ang, x, y) is ", best_offset, " with score ", max_score, min_variance
    
        # get the best angular offset
        return best_offset[0], best_offset[1:]

    def pressure_score_in_window(self, p_map, idx, window_size):

        n = idx[0]
        m = idx[1]

        l = 1
        if window_size%2 == 0:
            l = window_size/2
        else:
            l = (window_size-1)/2

        count = 0
        score_l  = []
        for i in range(n-l, n+l+1):
            for j in range(m-l, m+l+1):
                if i >=0 and i<len(p_map) and j>=0 and j<len(p_map[0]):

                    x = n-i
                    y = m-j
                    dist = float(2.0*window_size -x -y)/float(2.0*window_size) #np.sqrt(float(x*x+y*y))
                    score_l.append(p_map[i,j] * dist)
                    count = count + 1

        return np.array([np.mean(score_l), np.std(score_l)])
        
    def run(self):
        '''Uses the Rotation, translation, and slices obtained in
        initialization to create a synthetic database of images and ground 
        truth values'''

if __name__ == "__main__":
    #Initialize trainer with a training database file
    training_database_pkl_directory = sys.argv[1] #path to the training database  
    p = DatabaseCreator(training_database_pkl_directory) 
    #p.run()
