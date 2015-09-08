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

        self.training_dump_path = training_database_pkl_directory
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

        self.ang_offset   = None
        self.trans_offset = None
        ([self.split_matrices, self.split_targets]) = (
                                    self.preprocess_home_position(
                                    home_sup_pressure_map, home_sup_joint_pos))

        ## self.pca_transformation_sup(home_sup_pressure_map, home_sup_joint_pos)
        

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
        self.ang_offset, self.trans_offset = self.getOffset(rot_trans_targets_mat, rotated_p_map)
        rot_trans_targets_mat = np.dot(np.asarray(rot_trans_targets_mat),
                                       np.array([[np.cos(self.ang_offset), -np.sin(self.ang_offset)],
                                                 [np.sin(self.ang_offset), np.cos(self.ang_offset)]]))
        rot_trans_targets_mat = rot_trans_targets_mat + self.trans_offset        
        
        
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
        #self.visualize_pressure_map(rotated_p_map)
        #plt.show()
        
        #Now we slice the image into parts
        #Choose the lowest between the left and right hand
        upper_lower_torso_cut = max(rotated_target_coord[4][0],
                                rotated_target_coord[5][0]) + 3
        #Central line is through the torso
        left_right_side_cut =  rotated_target_coord[1][1]
        #Cut 3 pixels below the head marker
        head_horz_cut = rotated_target_coord[0][0] + 5
        head_vert_cut = ([rotated_target_coord[0][1] - 4 ,
                          rotated_target_coord[0][1] + 4]) 
        
        template_image = np.zeros(self.mat_size)
        template_target = np.zeros(np.shape(rot_trans_targets_mat))
        #Head Slice 
        slice_0 = np.copy(template_image)
        target_slice_0 = template_target[:]
        slice_0[:head_horz_cut, head_vert_cut[0]:head_vert_cut[1]] = 100.0 
        target_slice_0[0] += 1.0 
        #Right Arm Slice 
        slice_1 = np.copy(template_image)
        target_slice_1 = template_target[:]
        slice_1[:upper_lower_torso_cut, :left_right_side_cut] = 100.0
        slice_1[:head_horz_cut, head_vert_cut[0]:left_right_side_cut] = 0
        #target_slice_1[1] = target_slice_1[1] + 1.0 
        target_slice_1[1:3] += 1.0
        #Left Arm Slice 
        slice_2 = np.copy(template_image)
        target_slice_2 = template_target[:]
        slice_2[:upper_lower_torso_cut, left_right_side_cut + 1:] = 100.0
        slice_2[:head_horz_cut, left_right_side_cut:head_vert_cut[1]] = 0
        target_slice_2[4:5] += 1.0
        #Right leg Slice 
        slice_3 = np.copy(template_image)
        target_slice_3 = template_target[:]
        slice_3[upper_lower_torso_cut:, :left_right_side_cut] = 100.0
        target_slice_3[6:7] += 1.0
        #Left leg Slice 
        slice_4 = template_image[:] 
        target_slice_4 = np.copy(template_target)       
        slice_4[upper_lower_torso_cut:, left_right_side_cut + 1:] = 100.0
        target_slice_4[8:9] += 1.0

        image_slices = [slice_0, slice_1, slice_2, slice_3, slice_4]
        target_slices = ([target_slice_0, 
                          target_slice_1, 
                          target_slice_2, 
                          target_slice_3, 
                          target_slice_4])
        return image_slices, target_slices



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
        

    def pca_transformation_sup(self, p_map_raw, target_raw):
        '''Perform PCA and any additional rotations and translations that we 
        made to the home image'''

        # map translation and rotation ------------------------------------------------

        #Reshape to create 2D pressure map
        p_map = np.asarray(np.reshape(p_map_raw, self.mat_size))
        #Get the nonzero indices
        nzero_indices = np.nonzero(p_map)
        #Perform PCA on the non-zero elements of the pressure map
        pca_x_tuples = zip(nzero_indices[1], 
                                    nzero_indices[0]*(-1) + (NUMOFTAXELS_X-1))
        pca_x_pixels = np.asarray([list(elem) for elem in pca_x_tuples])
        pca_y_pixels = [p_map[elem] for elem in zip(nzero_indices[0],
            nzero_indices[1])]

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
        for i in range(len(pca_y_pixels)):
            rotated_p_map[rotated_p_map_coord[i]] = pca_y_pixels[i]/2.0
            
        # target translation and rotation ---------------------------------------------
        target_raw_2d = self.preprocess_targets(target_raw) 
        #Targets in the mat frame
        target_raw = (
                [self.world_to_mat(elem) for elem in target_raw_2d])

        #We need only X,Y coordinates in the mat frame
        targets_mat = np.asarray([[elem[0], elem[1]] for elem in target_raw])

        #The output of PCA needs rotation by -90 
        rot_targets_mat = self.pca_mat.transform(targets_mat)
        rot_targets_mat = np.dot(np.asarray(rot_targets_mat),
                                                np.array([[0, -1],[-1, 0]]))

        ## print rot_targets_mat 
        rot_trans_targets_mat = np.asarray(
            [np.asarray(elem) + 
            INTER_SENSOR_DISTANCE*np.array([NUMOFTAXELS_Y/2, NUMOFTAXELS_X/2]) 
            for elem in rot_targets_mat]) 
        
        transformed_target = np.dot(np.asarray(rot_trans_targets_mat),
                                       np.array([[np.cos(self.ang_offset), -np.sin(self.ang_offset)],
                                                 [np.sin(self.ang_offset), np.cos(self.ang_offset)]]))
        transformed_target = transformed_target + self.trans_offset        


        ## print rot_trans_targets_mat
        rot_trans_targets_pixels = ([self.mat_to_taxels(elem) for elem in
                transformed_target]) 

        rotated_target_coord = ([tuple([(-1)*(elem[1] - (NUMOFTAXELS_X - 1)), 
                                    elem[0]]) for elem in rot_trans_targets_pixels])

        for i in range(len(rotated_target_coord)):
            rotated_p_map[rotated_target_coord[i]] = 100
        
        #self.visualize_pressure_map(rotated_p_map)
        #plt.show()
        

        return rotated_p_map, transformed_target
        
    def run(self):
        '''Uses the Rotation, translation, and slices obtained in
        initialization to create a synthetic database of images and ground 
        truth values'''
        head_sup = pkl.load(
                open(training_database_pkl_directory+'head_sup.p', "rb")) 
        RH_sup = pkl.load(
                open(training_database_pkl_directory+'RH_sup.p', "rb")) 
        LH_sup = pkl.load(
                open(training_database_pkl_directory+'LH_sup.p', "rb")) 
        RL_sup = pkl.load(
                open(training_database_pkl_directory+'RL_sup.p', "rb")) 
        LL_sup = pkl.load(
                open(training_database_pkl_directory+'LL_sup.p', "rb")) 
        del head_sup['mat_o']
        del RH_sup['mat_o']
        del LH_sup['mat_o']
        del RL_sup['mat_o']
        del LL_sup['mat_o']
        #Slice each image using the slices computed earlier
        head_sliced = {}
        RH_sliced = {}
        LH_sliced = {}
        RL_sliced = {}
        LL_sliced = {}

        for p_map_raw in head_sup.keys():
                target_raw = head_sup[p_map_raw]
                [rotated_p_map, rotated_target] = self.pca_transformation_sup(
                                            p_map_raw, target_raw)
                sliced_p_map = np.multiply(rotated_p_map,
                        self.split_matrices[0])
                sliced_target = np.multiply(rotated_target,
                        self.split_targets[0])
                head_sliced[tuple(sliced_p_map.flatten())] = sliced_target


        for p_map_raw in RH_sup.keys():
                target_raw = RH_sup[p_map_raw]
                [rotated_p_map, rotated_target] = self.pca_transformation_sup(
                                            p_map_raw, target_raw)
                sliced_p_map = np.multiply(rotated_p_map,
                        self.split_matrices[1])
                sliced_target = np.multiply(rotated_target,
                        self.split_targets[1])
                RH_sliced[tuple(sliced_p_map.flatten())] = sliced_target

        for p_map_raw in LH_sup.keys():
                target_raw = LH_sup[p_map_raw]
                [rotated_p_map, rotated_target] = self.pca_transformation_sup(
                                            p_map_raw, target_raw)
                sliced_p_map = np.multiply(rotated_p_map,
                        self.split_matrices[2])
                sliced_target = np.multiply(rotated_target,
                        self.split_targets[2])
                LH_sliced[tuple(sliced_p_map.flatten())] = sliced_target

        for p_map_raw in RL_sup.keys():
                target_raw = RL_sup[p_map_raw]
                [rotated_p_map, rotated_target] = self.pca_transformation_sup(
                                            p_map_raw, target_raw)
                sliced_p_map = np.multiply(rotated_p_map,
                        self.split_matrices[3])
                sliced_target = np.multiply(rotated_target,
                        self.split_targets[3])
                RL_sliced[tuple(sliced_p_map.flatten())] = sliced_target

        for p_map_raw in LL_sup.keys():
                target_raw = LL_sup[p_map_raw]
                [rotated_p_map, rotated_target] = self.pca_transformation_sup(
                                            p_map_raw, target_raw)
                sliced_p_map = np.multiply(rotated_p_map,
                        self.split_matrices[4])
                sliced_target = np.multiply(rotated_target,
                        self.split_targets[4])
                LL_sliced[tuple(sliced_p_map.flatten())] = sliced_target

        final_database = {}
        for head_p_map in head_sliced.keys():
            for RH_p_map in RH_sliced.keys():
                for LH_p_map in LH_sliced.keys():
                    for RL_p_map in RL_sliced.keys():
                        for LL_p_map in LL_sliced.keys():
                            final_p_map = (np.asarray(head_p_map) + 
                                           np.asarray(RH_p_map) + 
                                           np.asarray(LH_p_map) + 
                                           np.asarray(RL_p_map) + 
                                           np.asarray(LL_p_map))
                            final_target = (np.asarray(head_sliced[head_p_map])+
                                            np.asarray(RH_sliced[RH_p_map]) + 
                                            np.asarray(LH_sliced[LH_p_map]) + 
                                            np.asarray(RL_sliced[RL_p_map]) +
                                            np.asarray(LL_sliced[LL_p_map]))
                            final_database[tuple(final_p_map)] = final_target
        
        pkl.dump(final_database, 
                open(self.training_dump_path+'final_database.p', 'wb'))

if __name__ == "__main__":
    #Initialize trainer with a training database file
    training_database_pkl_directory = sys.argv[1] #path to the training database  
    p = DatabaseCreator(training_database_pkl_directory) 
    p.run()
