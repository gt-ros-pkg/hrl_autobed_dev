#!/usr/bin/env python
import sys
import numpy as np
import math

import roslib; roslib.load_manifest('autobed_physical_trainer')
import rospy



def generateWeightedData(data_set, gaussian_params)
    '''
    Input
    data_set: A list of the pairs of pressure mat and corresponding marker array. 
              The type of of mat and marker array is list, array, or matrix. In detail,
              The size of mat will be N-by-M and the size of marker array will be 3-by-K.
    gaussian_params: The list of means and variances of two dimensional gaussian distributions.
                     Each element of the list contains a set of gaussian parameters for a pair 
                     of pressure mat and marker array such that

                     [[[mean11, mean12, ...],[var11, var12, ...]], 
                      [[mean21, mean22, ...],[var21, var22, ...]], 
                      ... ]

                     The mean variable contains two elements such as
                     [mean_x, mean_y]

                     The var variable contains two elements such as
                     [var_x, var_y]                                          

    Output
    weighted_data_set: A list of the pairs of pressure mat and correspoding marker array.
    '''

    weighted_data_set = []
    for idx, data in enumerate(data_set):

        # Get size of pressure mat
        pressure_mat = data[0]
        marker_array = data[1]
        n,m = np.shape(pressure_mat)

        # Get weight parameter set
        params = gaussian_params[idx]

        # Define container for each data
        weighted_data = []
        
        # Multiply pressure mat with gaussian-weighted region
        for i in xrange(len(params[0])):

            # Get gaussian parameter
            mu  = params[0][i]
            var = params[1][i]

            # Get gaussian muliplier
            mat = np.zeros((n,m))

            # Multiplication
            weighted_pressure_mat = 
            weighted_marker_array = 

            weighted_data.append([weighted_pressure_mat, weighted_marker_array])
            
        weighted_data_set.append(weighted_data)

    return weighted_data_set
