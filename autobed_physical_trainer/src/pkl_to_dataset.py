#!/usr/bin/env python
import sys
import os
import numpy as np
import cPickle as pkl
import random

# ROS
import roslib; roslib.load_manifest('autobed_physical_trainer')

# Graphics
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Machine Learning
from scipy import ndimage
## from skimage import data, color, exposure
from sklearn.decomposition import PCA

# HRL libraries
import hrl_lib.util as ut


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
    def __init__(self, training_database_pkl_directory, save_pdf=False, verbose=False):

        # Set initial parameters
        self.training_dump_path = training_database_pkl_directory.rstrip('/')
        self.save_pdf           = save_pdf
        self.verbose            = verbose

        home_sup_dat = pkl.load(
                open(os.path.join(self.training_dump_path,'home_sup.p'), "r"))         
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
        if self.verbose: print "Checking database for empty values."
        empty_count = 0
        for dict_entry in list(home_sup_dat.keys()):
            if len(home_sup_dat[dict_entry]) < (33) or (len(dict_entry) <
                    self.mat_size[0]*self.mat_size[1]):
                empty_count += 1
                del home_sup_dat[dict_entry]
        if self.verbose: print "Empty value check results: {} rogue entries found".format(
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
        orig_p_map = np.asarray(np.reshape(p_map_flat, self.mat_size))
        orig_targets = target[:]

        #Perform PCA on the pressure map to rotate and translate it to a known
        #value
        rotated_p_map = self.rotate_taxel_space(orig_p_map)
        #Perform PCA on the 3D target values to rotate and translate the 
        #targets
        rotated_targets = self.rotate_3D_space(orig_p_map, orig_targets)

        #Run matching function to find the best rotation offset
        self.ang_offset, self.trans_offset = self.getOffset(
                                        rotated_targets, rotated_p_map)
        rotated_targets = np.dot(np.asarray(rotated_targets),
                np.array([[np.cos(self.ang_offset), -np.sin(self.ang_offset)],
                [np.sin(self.ang_offset), np.cos(self.ang_offset)]]))
        rotated_targets += self.trans_offset        
        
        ## print rot_trans_targets_mat
        rotated_targets_pixels = ([self.mat_to_taxels(elem) for elem in
                rotated_targets]) 

        rotated_target_coord = ([tuple([(-1)*(elem[1] - (NUMOFTAXELS_X - 1)), 
                                elem[0]]) for elem in rotated_targets_pixels])

        #Slice up the pressure map values from rotated target values projected
        #in the taxel space
        [p_map_slices, target_slices] = (
                                self.slice_pressure_map(rotated_target_coord))
        ## self.visualize_pressure_map_slice(p_map_flat, rotated_p_map,
        ##         rotated_p_map, targets_raw=orig_targets, rotated_targets=rotated_targets)
        
        return p_map_slices, target_slices


    def rotate_3D_space(self, p_map, target):
        ''' Rotate the 3D target values (the 3D position of the markers
        attached to subject) using PCA'''
        #Get the indices of the pressure map that have non-zero pressure value
        nzero_indices = np.nonzero(p_map)
        #Perform PCA on the non-zero elements of the pressure map
        pca_x_tuples = zip(nzero_indices[1], 
                                    nzero_indices[0]*(-1) + (NUMOFTAXELS_X-1))
        pca_x_pixels = np.asarray([list(elem) for elem in pca_x_tuples])
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
        
        #Translate the targets to the center of the mat so that they match the 
        #pressure map
        rot_trans_targets_mat = np.asarray(
            [np.asarray(elem) + 
            INTER_SENSOR_DISTANCE*np.array([NUMOFTAXELS_Y/2, NUMOFTAXELS_X/2]) 
            for elem in rot_targets_mat]) 

        return rot_trans_targets_mat


    def rotate_taxel_space(self, p_map):
        '''Rotates pressure map given to it using PCA and translates it back to 
        center of the pressure mat.'''
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
        
        # Daehyung: NUMOFTAXELS_Y/2 uses round-off (correct?)        
        rot_trans_x_pixels = np.asarray(
            [np.asarray(elem) + np.array([NUMOFTAXELS_Y/2, NUMOFTAXELS_X/2]) # temp
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
            rotated_p_map[rotated_p_map_coord[i]] = pca_y_pixels[i]
        return rotated_p_map


    def slice_pressure_map(self, coord):        
        '''Slices Pressure map and coordinates into parts.
        1. Head Part
        2. Right Hand
        3. Left Hand
        4. Right Leg
        5. Left Leg
        Returns: Image Templates that are then multiplied to pressure map
        to produce better output.
        '''
        #Choose the lowest(max) between the left and right hand
        upper_lower_torso_cut = max(coord[5][0],
                                coord[6][0]) + 7
        #Central line is through the torso
        #left_right_side_cut =  rotated_target_coord[1][1]
        left_right_side_cut =  np.floor(NUMOFTAXELS_Y/2)
        #Cut 3 pixels below the head marker
        head_horz_cut = min(coord[1][0], coord[2][0]) - 2 
        head_vert_cut = ([coord[1][1] + 2 ,
                          coord[2][1] - 2]) 
        
        template_image = np.zeros(self.mat_size)
        template_target = np.zeros(np.shape(coord))
        #Head Slice 
        slice_0 = np.copy(template_image)
        target_slice_0 = template_target[:]
        slice_0[:head_horz_cut, head_vert_cut[0]:head_vert_cut[1]] = 1.0 
        target_slice_0[0] += 1.0 
        #Right Arm Slice 
        slice_1 = np.copy(template_image)
        target_slice_1 = template_target[:]
        slice_1[:upper_lower_torso_cut, :left_right_side_cut] = 1.0
        slice_1[:head_horz_cut, head_vert_cut[0]:left_right_side_cut] = 0
        #target_slice_1[1] = target_slice_1[1] + 1.0 
        target_slice_1[1:3] += 1.0
        #Left Arm Slice 
        slice_2 = np.copy(template_image)
        target_slice_2 = template_target[:]
        slice_2[:upper_lower_torso_cut, left_right_side_cut:] = 1.0
        slice_2[:head_horz_cut, left_right_side_cut:head_vert_cut[1]] = 0
        target_slice_2[4:5] += 1.0
        #Right leg Slice 
        slice_3 = np.copy(template_image)
        target_slice_3 = template_target[:]
        slice_3[upper_lower_torso_cut:, :left_right_side_cut] = 1.0
        target_slice_3[6:7] += 1.0
        #Left leg Slice 
        slice_4 = template_image[:] 
        target_slice_4 = np.copy(template_target)       
        slice_4[upper_lower_torso_cut:, left_right_side_cut:] = 1.0
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


    def visualize_pressure_map(self, pressure_map_matrix, rotated_targets=None, fileNumber=0):
        '''Visualizing a plot of the pressure map'''
        fig = plt.figure()
        plt.imshow(pressure_map_matrix, interpolation='nearest', cmap=
                plt.cm.bwr, origin='upper', vmin=0, vmax=100)

        if rotated_targets is not None:
            rot_trans_targets_pixels = ([self.mat_to_taxels(elem) for elem in
                                         rotated_targets]) 
                
            rotated_target_coord = ([tuple([(-1)*(elem[1] - (NUMOFTAXELS_X - 1)), 
                                            elem[0]]) for elem in rot_trans_targets_pixels])

            for i in range(len(rotated_target_coord)):
                ## rotated_p_map[rotated_target_coord[i]] = 100
                plt.plot([float(rotated_target_coord[i][1])], [float(rotated_target_coord[i][0])],\
                         'y*', ms=10)
                        
        if self.save_pdf == True: 
            print "Visualized pressure map ", fileNumber                                        
            fig.savefig('test_'+str(fileNumber)+'.pdf')
            os.system('mv test*.p* ~/Dropbox/HRL/') # only for Daehyung
            plt.close()
        else:
            plt.show()
        
        return

    def visualize_pressure_map_slice(self, p_map_raw, rotated_p_map, sliced_p_map, \
                                     targets_raw=None, rotated_targets=None, sliced_targets=None, \
                                     fileNumber=0):
        p_map = np.asarray(np.reshape(p_map_raw, self.mat_size))
        fig = plt.figure()
        ax = fig.add_subplot(1, 3, 1)
        ax.imshow(p_map, interpolation='nearest', cmap=
                        plt.cm.bwr, origin='upper', vmin=0, vmax=100)
        ax1 = fig.add_subplot(1, 3, 2)
        ## ax1.imshow(rotated_p_map, interpolation='nearest', cmap=
        ##                 plt.cm.bwr, origin='upper', vmin=0, vmax=100)        
        ax2 = fig.add_subplot(1, 3, 3)
        ax2.imshow(sliced_p_map, interpolation='nearest', cmap=
                        plt.cm.bwr, origin='upper', vmin=0, vmax=100)



        if targets_raw is not None:
            print np.shape(targets_raw)
            n = len(targets_raw)
            targets_mat = np.asarray([[elem[0], elem[1]] for elem in
                targets_raw])
            ## targets_mat = np.dot(np.asarray(targets_mat),
            ##                      np.array([[0, -1],[-1, 0]]))           
            targets_pixels = ([self.mat_to_taxels(elem) for elem in
                                          targets_mat]) 
            target_coord = np.array(([tuple([(-1)*(elem[1] - (NUMOFTAXELS_X - 1)), 
                                             elem[0]]) for elem in targets_pixels]))
            ax.plot(target_coord[:,1], target_coord[:,0], 'k*', ms=8)
            
        if rotated_targets is not None:

            rot_trans_targets_pixels = ([self.mat_to_taxels(elem) for elem in
                                         rotated_targets]) 
                
            rotated_target_coord = ([tuple([(-1)*(elem[1] - (NUMOFTAXELS_X - 1)), 
                                            elem[0]]) for elem in rot_trans_targets_pixels])

            for i in range(len(rotated_target_coord)):
                ## rotated_p_map[rotated_target_coord[i]] = 100
                ax1.plot([float(rotated_target_coord[i][1])], [float(rotated_target_coord[i][0])],\
                         'y*', ms=10)
                

            ax1.imshow(rotated_p_map, interpolation='nearest', cmap=
                       plt.cm.bwr, origin='upper', vmin=0, vmax=100)        
            ## ax1.plot(rotated_target_coord[:,0], rotated_target_coord[:,1], 'k*', ms=8)
            ## ax1.plot(rot_trans_targets_pixels[:,0], rot_trans_targets_pixels[:,1], 'k*', ms=8)

        if sliced_targets is not None:
            print "under construction"
            
        
        if self.save_pdf == True: 
            print "Visualized pressure map ", fileNumber                                        
            fig.savefig('test_'+str(fileNumber)+'.png')
            os.system('mv test*.p* ~/Dropbox/HRL/') # only for Daehyung
            plt.close()
        else:
            plt.show()

        return
        
    def getOffset(self, target_mat, p_map, plot=False):
        '''Find the best angular and translation offset''' 

        iteration   = 500
        ang_offset  = 0.0
        ang_range   = [0.0, 10.0/180.0*np.pi]
        x_range     = [0.0, 0.15]
        y_range     = [0.0, 0.15]
        max_score   = 0.
        min_variance = 1000.0
        best_offset = np.array([0.,0.,0.]) #ang, x, y
        window_size = 3

        map_pressure_thres = 10.0
        head_pixel_range  = [0,10]
        ankle_pixel_range = [-5,-0]
        
        # get head and food parts in map
        part_map = np.zeros(np.shape(p_map))
        for i in xrange(len(p_map)):
            for j in xrange(len(p_map[i])):
                if i>=head_pixel_range[0] and i<=head_pixel_range[1] and \
                  p_map[i,j] > map_pressure_thres:
                    part_map[i,j] = 50.0
                if i>len(p_map)+ankle_pixel_range[0] and i < len(p_map)+ankle_pixel_range[1] \
                  and p_map[i,j] > map_pressure_thres:
                    part_map[i,j] = 50.0        
        ## part_map[0:13,:] = p_map[0:13,:]
        ## part_map[-5:-1,:] = p_map[-5:-1,:]
        #p_map = part_map        
        if plot: self.visualize_pressure_map(p_map)
        
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
                    dist = float(2.0*window_size -x -y)/float(2.0*window_size) 
                    #np.sqrt(float(x*x+y*y))
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
            rotated_p_map[rotated_p_map_coord[i]] = pca_y_pixels[i]
            
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
                                       np.array([[np.cos(self.ang_offset), \
                                                  -np.sin(self.ang_offset)],\
                                                 [np.sin(self.ang_offset), \
                                                  np.cos(self.ang_offset)]]))
        transformed_target = transformed_target + self.trans_offset        


        ## print rot_trans_targets_mat
        rot_trans_targets_pixels = ([self.mat_to_taxels(elem) for elem in
                transformed_target]) 

        rotated_target_coord = ([tuple([(-1)*(elem[1] - (NUMOFTAXELS_X - 1)), 
                                    elem[0]]) for elem in rot_trans_targets_pixels])

        #for i in range(len(rotated_target_coord)):
            #rotated_p_map[rotated_target_coord[i]] = 100
        
        #self.visualize_pressure_map(rotated_p_map)
        #plt.show()
        

        return rotated_p_map, transformed_target
        
    def run(self):
        '''Uses the Rotation, translation, and slices obtained in
        initialization to create a synthetic database of images and ground 
        truth values'''
        home_sup = pkl.load(
                open(os.path.join(self.training_dump_path,'home_sup.p'), "rb")) 
        head_sup = pkl.load(
                open(os.path.join(self.training_dump_path,'head_sup.p'), "rb")) 
        RH_sup = pkl.load(
                open(os.path.join(self.training_dump_path,'RH_sup.p'), "rb")) 
        LH_sup = pkl.load(
                open(os.path.join(self.training_dump_path,'LH_sup.p'), "rb")) 
        RL_sup = pkl.load(
                open(os.path.join(self.training_dump_path,'RL_sup.p'), "rb")) 
        LL_sup = pkl.load(
                open(os.path.join(self.training_dump_path,'LL_sup.p'), "rb")) 
        try:
            del home_sup['mat_o']
            del head_sup['mat_o']
            del RH_sup['mat_o']
            del LH_sup['mat_o']
            del RL_sup['mat_o']
            del LL_sup['mat_o']
        except KeyError:
            pass
        #Slice each image using the slices computed earlier
        head_sliced = {}
        RH_sliced = {}
        LH_sliced = {}
        RL_sliced = {}
        LL_sliced = {}

        ## count = 0                
        # map_raw: pressure map
        # target_raw: marker 
        p_map_raw = home_sup.keys()[0]
        target_raw = home_sup[p_map_raw]
        [rotated_p_map, rotated_target] = self.pca_transformation_sup(
                                    p_map_raw, target_raw)
        sliced_p_map = np.multiply(rotated_p_map,
                self.split_matrices[0])
        sliced_target = np.multiply(rotated_target,
                self.split_targets[0])
        head_sliced[tuple(sliced_p_map.flatten())] = sliced_target
        sliced_p_map = np.multiply(rotated_p_map,
                self.split_matrices[1])
        sliced_target = np.multiply(rotated_target,
                self.split_targets[1])
        RH_sliced[tuple(sliced_p_map.flatten())] = sliced_target
        sliced_p_map = np.multiply(rotated_p_map,
                self.split_matrices[2])
        sliced_target = np.multiply(rotated_target,
                self.split_targets[2])
        LH_sliced[tuple(sliced_p_map.flatten())] = sliced_target
        sliced_p_map = np.multiply(rotated_p_map,
                self.split_matrices[3])
        sliced_target = np.multiply(rotated_target,
                self.split_targets[3])
        RL_sliced[tuple(sliced_p_map.flatten())] = sliced_target
        sliced_p_map = np.multiply(rotated_p_map,
                self.split_matrices[4])
        sliced_target = np.multiply(rotated_target,
                self.split_targets[4])
        LL_sliced[tuple(sliced_p_map.flatten())] = sliced_target

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
                
        for i, p_map_raw in enumerate(LL_sup.keys()):
                target_raw = LL_sup[p_map_raw]
                [rotated_p_map, rotated_target] = self.pca_transformation_sup(
                                            p_map_raw, target_raw)
                sliced_p_map = np.multiply(rotated_p_map,
                        self.split_matrices[4])
                sliced_target = np.multiply(rotated_target,
                        self.split_targets[4])
                LL_sliced[tuple(sliced_p_map.flatten())] = sliced_target
                #print len(LL_sup.keys())

        for p_map_raw in RL_sup.keys():
                target_raw = RL_sup[p_map_raw]
                [rotated_p_map, rotated_target] = self.pca_transformation_sup(
                                            p_map_raw, target_raw)
                sliced_p_map = np.multiply(rotated_p_map,
                        self.split_matrices[3])
                sliced_target = np.multiply(rotated_target,
                        self.split_targets[3])
                RL_sliced[tuple(sliced_p_map.flatten())] = sliced_target

                targets_reshaped = self.preprocess_targets(target_raw)
                ## self.visualize_pressure_map_slice(p_map_raw, rotated_p_map, sliced_p_map, \
                ##                                   targets_raw=targets_reshaped, \
                ##                                   rotated_targets=rotated_target, \
                ##                                   sliced_targets=sliced_target, fileNumber=i)
                                

                
        # temp
        count = 0
        zoom_factor=2.0
        final_database = {}
        for head_p_map in head_sliced.keys():
            for RH_p_map in RH_sliced.keys():
                for LH_p_map in LH_sliced.keys():
                    for RL_p_map in RL_sliced.keys():
                        for LL_p_map in LL_sliced.keys():
                            stitched_p_map = (np.asarray(head_p_map) + 
                                           np.asarray(RH_p_map) + 
                                           np.asarray(LH_p_map) + 
                                           np.asarray(RL_p_map) + 
                                           np.asarray(LL_p_map))
                            final_target = (np.asarray(head_sliced[head_p_map])+
                                            np.asarray(RH_sliced[RH_p_map]) + 
                                            np.asarray(LH_sliced[LH_p_map]) + 
                                            np.asarray(RL_sliced[RL_p_map]) +
                                            np.asarray(LL_sliced[LL_p_map]))

                            print final_target
                            print RL_sliced[RL_p_map]
                            print LL_sliced[LL_p_map]
                            sys.exit()

                            final_p_map = ndimage.zoom(
                                    np.reshape(stitched_p_map, self.mat_size), 
                                    zoom_factor, order=1)

                    self.visualize_pressure_map(final_p_map, rotated_targets=final_target*\
                                                zoom_factor)
                    sys.exit()
                            ##                             fileNumber=count)
                            
                            ## if count > 20: sys.exit()
                            ## else: count += 1
                            
                    final_database[tuple(final_p_map.flatten())] = final_target

        print "Save final_database"
        ## pkl.dump(final_database, 
        ##         open(os.path.join(self.training_dump_path,'final_database.p'), 'wb'))


if __name__ == "__main__":

    import optparse
    p = optparse.OptionParser()

    p.add_option('--training_data_path', '--path',  action='store', type='string', \
                 dest='trainingPath',\
                 default='~/hrl_file_server/autobed/pose_estimation_data/subject2_stitching_test/', \
                 help='Set path to the training database.')
    p.add_option('--save_pdf', '--sp',  action='store_true', dest='save_pdf',
                 default=False, help='Save plot as a pdf.')
    p.add_option('--verbose', '--v',  action='store_true', dest='verbose',
                 default=False, help='Printout everything (under construction).')
    
    opt, args = p.parse_args()
    
    
    #Initialize trainer with a training database file
    ## training_database_pkl_directory = sys.argv[1] #  
    p = DatabaseCreator(training_database_pkl_directory=opt.trainingPath,\
                        save_pdf=opt.save_pdf,\
                        verbose=opt.verbose) 
    p.run()
