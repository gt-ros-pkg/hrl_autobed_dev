#!/usr/bin/env python
import sys
import numpy as np
import math

import roslib; roslib.load_manifest('autobed_physical_trainer')
import rospy

from scipy.stats import multivariate_normal
import matplotlib
import matplotlib.pyplot as plt


def generateWeightedData(data_set, gaussian_params=None, verbose=False):
    '''
    Input
    data_set: A list of the pairs of pressure mat and corresponding marker array. 
              The type of of mat and marker array is list, array, or matrix. In detail,
              The size of mat will be N-by-M and the size of marker array will be 3-by-K.
    gaussian_params: The list of means and variances of two dimensional gaussian distributions.
                     Each element of the list contains a set of gaussian parameters for a pair 
                     of pressure mat and marker array such that

                     [[mean1,cov1],[mean2,cov2], ... ]

                     The mean variable contains two elements such as
                     [mean_x, mean_y]

                     The cov variable contains four elements as a square matrix such as
                     [[var_xx, var_xy],[var_yx, var_yy]]                                          

    Output
    weighted_data_set: A list of the pairs of pressure mat and correspoding marker array.
    '''
    if verbose: print "Start to generate weighted data"

    if gaussian_params is None:
        gaussian_params = []
        mean_list = [20,22]
        var_list = np.array([[30,7],[7,3]])
        gaussian_params.append([mean_list, var_list])
    
        
    weighted_data_set = []
    for idx, data in enumerate(data_set):

        # Get size of pressure mat
        pressure_mat = data[0]
        marker_array = data[1]
        n,m = np.shape(pressure_mat)

        # Define container for each data
        weighted_data = []
        
        # Multiply pressure mat with gaussian-weighted region
        for i in xrange(len(gaussian_params)):

            # Get gaussian parameter
            mu_list  = gaussian_params[i][0]
            var_list = gaussian_params[i][1]

            # Get gaussian muliplier
            mat = np.zeros((n,m))

            x_range = np.arange(0,n)
            y_range = np.arange(0,m)   
            x,y = np.meshgrid(x_range, y_range)
            
            pos = np.empty(x.shape + (2,))
            pos[:,:,0] = x; pos[:,:,1] = y
            rv  = multivariate_normal(mu_list, var_list)
            weight_mat = rv.pdf(pos).T
            
            ## plt.contourf(x,y,rv.pdf(pos))
            ## plt.axis('equal')
            ## plt.show()
            ## print np.shape(pressure_mat), np.shape(weight_mat)
            print np.shape(pressure_mat), np.shape(weight_mat)
            
            # Multiplication
            weighted_pressure_mat = pressure_mat * weight_mat
            weighted_marker_array = marker_array
            mat_viz(pressure_mat, weight_mat, weighted_pressure_mat)
            
            weighted_data.append([weighted_pressure_mat, weighted_marker_array])
            
        weighted_data_set.append(weighted_data)

    return weighted_data_set


def mat_viz(image, weight_image, weighted_image):
    fig = plt.figure()
    ax = fig.add_subplot(1,3,1)
    ax.imshow(image, interpolation='none', cmap=plt.cm.bwr, origin='upper', vmin=0, vmax=100)
    ax = fig.add_subplot(1,3,2)
    ax.imshow(weight_image, interpolation='none', cmap=plt.cm.bwr, origin='upper')
    ax = fig.add_subplot(1,3,3)
    ax.imshow(weighted_image, interpolation='none', cmap=plt.cm.bwr, origin='upper')
    
    ## ax.set_xlabel('x [pixels]')    
    ## ax.set_ylabel('y [pixels]')    
    ## plt.colorbar()    
    ## fig.savefig('test.pdf')    
    ## fig.savefig('test.png')    
    ## os.system('cp test.p* ~/Dropbox/HRL/')
    plt.show() 


if __name__ == "__main__":

    a = [[np.zeros((100,400)), np.zeros((2,7))]]
    generateWeightedData(a)
