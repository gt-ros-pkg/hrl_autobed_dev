#!/usr/bin/env python

import math, numpy as np
import sys, time, random

NAME = 'utm_python_listener'
import roslib; roslib.load_manifest('autobed_engine')
import rospy
from sensor_msgs.msg import LaserScan
import threading
from std_msgs.msg import Float32

import copy

import numpy as np

class HokuyoInterpreter():
    ''' This class has the data of a laser scan.
    '''
    def __init__(self, start_angle, end_angle):
        '''
            angular_res - angle (radians) between consecutive points in the scan.
            min_range, max_range -  in meters
            ranges,angles,intensities - 1xn_points numpy matrix.
        '''
        min_ang_degrees = -90#rospy.get_param(hokuyo_node_name+'/min_ang_degrees')
        max_ang_degrees = 90#rospy.get_param(hokuyo_node_name+'/max_ang_degrees')
        start_angle_fullscan = math.radians(min_ang_degrees)
        end_angle_fullscan = math.radians(max_ang_degrees)
        self.angular_res = math.radians(0.25)
        if start_angle == None or start_angle<start_angle_fullscan:
            start_angle_subscan = start_angle_fullscan
        else:
            start_angle_subscan = start_angle
        if end_angle == None or end_angle>end_angle_fullscan:
            end_angle_subscan = end_angle_fullscan
        else:
            end_angle_subscan = end_angle
        self.start_index_fullscan=int(round((start_angle_subscan-start_angle_fullscan)/self.angular_res))
        self.end_index_fullscan=int(round((end_angle_subscan-start_angle_fullscan)/self.angular_res))
        self.max_range   = 10.0
        self.min_range   = 0.1
        self.start_angle = start_angle_subscan
        self.end_angle   = end_angle_subscan
        self.n_points    = int((end_angle-start_angle)/self.angular_res)+1
        self.BED_HT_COEFF = 100
        self.angles = []
        for i in xrange(self.n_points):
            self.angles.append(self.index_to_angle(i))
        self.angles = np.array(self.angles)
        self.frame_lock = threading.RLock()
        self.connected_to_ros = False
        self.ranges = None

        rospy.Subscriber("/autobed/height_hokuyo/scan", LaserScan,
                         self.laserscan_callback, queue_size=1)
        self.pose_pub = rospy.Publisher("/bed_ht", Float32, latch=True)

    def bed_ht_from_scan(self):
        ''' Computes bed height from hokuyo scan'''
        inliers_array = []
        sample_count = 0
        COUNT_THRESH = 50
        ERR_THRESH = 0.0025
        NUM_INLIERS_THRESH = 100
        max_num_inliers = 0
        max_inliers_array = []
        ht_offset = 0.262953430414-0.0254
        i = random.randint(0, self.n_points - 1)
        while sample_count < COUNT_THRESH:
            num_inliers = 0
            inliers_array = []
            sample_count += 1
            ht_sample = math.cos(self.angles[i])*(self.ranges[i])
            for i in xrange(self.n_points):
                ht_est = math.cos(self.angles[i])*(self.ranges[i])
                ht_err = abs(ht_sample - ht_est)
                if ht_err < ERR_THRESH:
                    num_inliers += 1
                    inliers_array.append(ht_est)
            if num_inliers >  NUM_INLIERS_THRESH:
                p = (np.mean(inliers_array) - ht_offset)
                self.pubPose(p)
                return
            elif num_inliers > max_num_inliers:
                max_num_inliers = num_inliers
                max_inliers_array = inliers_array[:]
        p = (np.mean(max_inliers_array) - ht_offset)
        if math.isnan(p):
            print max_num_inliers
        if p<0.:
            p = 0.
        self.pubPose(p)

    def laserscan_callback(self, scan):
        with self.frame_lock:
            self.connected_to_ros = True
            self.ranges = np.array(scan.ranges[self.start_index_fullscan:self.end_index_fullscan+1])
            self.bed_ht_from_scan()

    def pubPose(self, dist):
        self.pose_pub.publish(dist*self.BED_HT_COEFF)
 
    def index_to_angle(self, index):
        ''' returns angle (radians) corresponding to index.
        '''
        return self.start_angle+index*self.angular_res

if __name__ == '__main__':
    rospy.init_node('bed_ht_estimation')
    start_angle = math.radians(-30)
    end_angle = math.radians(30)
    h = HokuyoInterpreter(start_angle, end_angle)
    print 'getting first scan'
    rate = rospy.Rate(10) # 25Hz, nominally.    
    while not rospy.is_shutdown():
        rate.sleep()



