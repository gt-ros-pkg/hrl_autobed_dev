#!/usr/bin/env python
import sys
import operator
import numpy as np
import matplotlib.pyplot as plt

import cPickle as pkl
import pressure_map_generator



def visualize_results(training_database_file, test_result_file):
    '''Will visualize the ground truth coordinates of the links of the human subject. Also will plot the estimated coordinates in the same plot'''
    pressure_mat = pressure_map_generator.MapGenerator()#Create a pressure mat object   
    closest_map = np.zeros(pressure_mat.get_mat_size_in_taxels()) #Zero out the present pressure map. We will populate this in a bit.
    try:
        training = pkl.load(open(training_database_file, "rb"))#Load the database of trained poses
        test_results = pkl.load(open(test_result_file, "rb"))#Load the database of trained poses
    except:
        print "ERROR: No database found at location specified by you. Please correct location and try again"

    for result in test_results:
        #Plot first, the exact pressure map obtained in the test
        try:
            closest_map = training[result['best_pose']]['pressure_map']
            closest_map = closest_map.astype(int)
        except:
            break

        estimated_taxels = [pressure_mat.coordinates_to_taxel_positions(coordinate_center) for coordinate_center in result['estimated_positions']]
        ground_truth_taxels = [pressure_mat.coordinates_to_taxel_positions(coordinate_center) for coordinate_center in result['ground_truth']]
        for i in range(len(estimated_taxels)):
            closest_map[estimated_taxels[i][0]][estimated_taxels[i][1]] = 5
        for i in range(len(ground_truth_taxels)):
            closest_map[ground_truth_taxels[i][0]][ground_truth_taxels[i][1]] = 10
        plt.figure()
        plt.imshow(closest_map, interpolation='nearest', cmap=plt.cm.hot, origin='upper', vmin=0, vmax=10)
        plt.show()
        _ = raw_input("Press [enter] to continue.") # wait for input from the user
        print "The mean error in cms is {}".format(result["average error"])
        plt.close()    # close the figure to show the next one.

if __name__ == "__main__":
    #Initialize pose trainer with the path to training database, and test results
    visualize_results(sys.argv[1], sys.argv[2])
 
