#!/usr/bin/env python
import numpy as np
import roslib; roslib.load_manifest('hrl_msgs'); roslib.load_manifest('tf')
import rospy
import tf
from hrl_msgs.msg import FloatArrayBare
from sensor_msgs.msg import JointState
from math import *
import operator
from scipy.signal import remez
from scipy.signal import lfilter

from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped, Point, Pose, PoseStamped 
from std_msgs.msg import ColorRGBA
from tf.transformations import quaternion_from_euler


MAT_WIDTH = 0.762 #metres
MAT_HEIGHT = 1.854 #metres
MAT_HALF_WIDTH = MAT_WIDTH/2 
NUMOFTAXELS_X = 64#73 #taxels
NUMOFTAXELS_Y = 27#30 
LOW_TAXEL_THRESH_X = 0
LOW_TAXEL_THRESH_Y = 0
HIGH_TAXEL_THRESH_X = (NUMOFTAXELS_X - 1) 
HIGH_TAXEL_THRESH_Y = (NUMOFTAXELS_Y - 1) 



class AutobedConverter():

    def __init__(self):
        rospy.init_node('autobed_state_publisher', anonymous = False)
        self.joint_pub = rospy.Publisher('joint_states', JointState)
        self.pressure_grid_pub = rospy.Publisher('pressure_grid', Marker)
        self.sendToRviz=tf.TransformBroadcaster()
        self.listener = tf.TransformListener() 
        rospy.Subscriber("/abdout0", FloatArrayBare, self.bed_pose_cb)
        #rospy.Subscriber("/camera_o/pose", TransformStamped, 
        #        self.camera_pose_cb)
        rospy.Subscriber("/fsascan", FloatArrayBare, 
                self.pressure_map_cb)

        #Initialize camera pose to standard position of Kinect in the test
        #chamber
        self.camera_p = (-0.1093, 1.108, 2.86)
        #self.camera_q = (0.27, -0.011, -0.958, 0.0975)
        self.camera_q = tuple(quaternion_from_euler(1.57, 1.57, 0.0))
        #Low pass filter design
        self.bed_height = 0
        self.bin_numbers = 5
        self.bin_numbers_for_leg_filter = 21
        self.collated_head_angle = np.zeros((self.bin_numbers, 1))
        self.collated_leg_angle = np.zeros((
            self.bin_numbers_for_leg_filter, 1))
        self.lpf = remez(self.bin_numbers, [0, 0.1, 0.25, 0.5], [1.0, 0.0])
        self.lpf_for_legs = remez(self.bin_numbers_for_leg_filter, 
                [0, 0.0005, 0.1, 0.5], [1.0, 0.0])
        self.pressuremap_flat = np.zeros((1, NUMOFTAXELS_X*NUMOFTAXELS_Y))
        #Publisher for Markers (can send them all as one marker message instead of an array because they're all spheres of the same size
        self.marker_pub=rospy.Publisher('visualization_marker', Marker)

        #Subscribers for each of the joint markers
        rospy.Subscriber("head_o/pose", TransformStamped, self.head_marker_cb)
        rospy.Subscriber("torso_o/pose", TransformStamped, self.torso_marker_cb)
        rospy.Subscriber("r_elbow_o/pose", TransformStamped, self.r_elbow_marker_cb)
        rospy.Subscriber("l_elbow_o/pose", TransformStamped, self.l_elbow_marker_cb)     
        rospy.Subscriber("r_hand_o/pose", TransformStamped, self.r_hand_marker_cb)
        rospy.Subscriber("l_hand_o/pose", TransformStamped, self.l_hand_marker_cb)
        rospy.Subscriber("r_knee_o/pose", TransformStamped, self.r_knee_marker_cb)
        rospy.Subscriber("l_knee_o/pose", TransformStamped, self.l_knee_marker_cb)
        rospy.Subscriber("r_ankle_o/pose", TransformStamped, self.r_ankle_marker_cb)
        rospy.Subscriber("l_ankle_o/pose", TransformStamped, self.l_ankle_marker_cb)


        #callback for markers
    def head_marker_cb(self, msg):
        self.head_pose=msg          
        self.compose_marker_msg()

    def torso_marker_cb(self, msg):
        self.torso_pose=msg
    
    def r_elbow_marker_cb(self, msg):
        self.r_elbow_pose=msg

    def l_elbow_marker_cb(self, msg):
        self.l_elbow_pose=msg

    def r_hand_marker_cb(self, msg):
        self.r_hand_pose=msg

    def l_hand_marker_cb(self, msg):
        self.l_hand_pose=msg

    def r_knee_marker_cb(self, msg):
        self.r_knee_pose=msg

    def l_knee_marker_cb(self, msg):
        self.l_knee_pose=msg

    def r_ankle_marker_cb(self, msg):
        self.r_ankle_pose=msg

    def l_ankle_marker_cb(self, msg):
        self.l_ankle_pose=msg

    def compose_marker_msg(self):
        self.marker_msg=Marker()
        self.marker_msg.header.frame_id="/world"
        self.marker_msg.header.stamp=rospy.Time.now()
        self.marker_msg.ns="motion_capture_trackables"
        self.marker_msg.id=0
        self.marker_msg.type=Marker.SPHERE_LIST #if numbers needed SPHERE_LIST=7
        self.marker_msg.action=Marker.ADD # if numbers needed ADD=0
        self.marker_msg.pose.position.x=0.0
        self.marker_msg.pose.position.y=0.0 #TODO: check this offset. It should line up with the pressure mat      
        self.marker_msg.pose.position.z=2.0
        self.marker_msg.pose.orientation.x=0.0        
        self.marker_msg.pose.orientation.y=0.0
        self.marker_msg.pose.orientation.z=0.0
        self.marker_msg.pose.orientation.w=1.0
        list_of_markers=[self.head_pose.transform.translation, self.torso_pose.transform.translation, 
                         self.r_elbow_pose.transform.translation, self.l_elbow_pose.transform.translation, 
                         self.r_hand_pose.transform.translation, self.l_hand_pose.transform.translation, 
                         self.r_knee_pose.transform.translation, self.l_knee_pose.transform.translation, 
                         self.r_ankle_pose.transform.translation, self.l_ankle_pose.transform.translation]   
        for i in list_of_markers:
            self.marker_msg.points.append(i)
            
        self.marker_msg.scale.x=0.05
        self.marker_msg.scale.y=0.05
        self.marker_msg.scale.z=0.05 
        self.marker_msg.color.a=1.0
        self.marker_msg.color.r=1.0 
        self.marker_msg.color.g=0.0
        self.marker_msg.color.b=0.0
        self.marker_pub.publish(self.marker_msg)

    #callback for the pose messages from the kinect in the testing chamber
    def camera_pose_cb(self, data): 
        x_offset = 0.1
        y_offset = 0.1
        z_offset = -0.2
        r_offset = 0.0
        p_offset = 0.0
        yaw_offset = 0.0
        self.camera_p = (data.transform.translation.x + x_offset,
                        data.transform.translation.y + y_offset,
                        data.transform.translation.z + z_offset)
        self.camera_q = (data.transform.rotation.x, data.transform.rotation.y,
                    data.transform.rotation.z, data.transform.rotation.w)
        camera_q = tuple(map(operator.add, self.camera_q, 
            tuple(quaternion_from_euler(r_offset, p_offset, yaw_offset)))) 


    #callback for the pose messages from the autobed
    def bed_pose_cb(self, data): 
        poses=np.asarray(data.data);
        
        self.bed_height = ((poses[1]/100) - 0.09) if (((poses[1]/100) - 0.09) 
                > 0) else 0
        head_angle = (poses[0]*pi/180)
        leg_angle = (poses[2]*pi/180 - 0.1)
        self.collated_head_angle = np.delete(self.collated_head_angle, 0)
        self.collated_head_angle = np.append(self.collated_head_angle,
                [head_angle])
        self.collated_leg_angle = np.delete(self.collated_leg_angle, 0)
        self.collated_leg_angle = np.append(self.collated_leg_angle,
                [leg_angle])
 

    def pressure_map_cb(self, data):
        '''This callback accepts incoming pressure map from 
        the Vista Medical Pressure Mat and sends it out. 
        Remember, this array needs to be binarized to be used'''
        self.pressuremap_flat = [data.data]
 

    def filter_data(self):
        '''Creates a low pass filter to filter out high frequency noise'''
        self.leg_filt_data = self.truncate(np.dot(self.lpf_for_legs,
                    self.collated_leg_angle))
        self.head_filt_data = np.dot(self.lpf, self.collated_head_angle)
        return


    def truncate(self, f):
        '''Truncates/pads a float f to 1 decimal place without rounding'''
        fl_as_str = "%.2f" %f
        return float(fl_as_str)


    def sample_bed_surface(self):
        '''Sample the surface of the bed into a grid, that represents the
        pressure mat. This will help us in visualizing the pressure mat on 
        rviz interface as a CUBE LIST'''
        delta_x = 0.2
        delta_y = 0
        delta_z = 1.0
        surface_grid = np.zeros((NUMOFTAXELS_X*NUMOFTAXELS_Y, 3))
        taxel_count = 0
        for i in range(NUMOFTAXELS_X):
            for j in range(NUMOFTAXELS_Y):
                surface_grid[taxel_count] = [(i*(MAT_HEIGHT/NUMOFTAXELS_X) + 
                                                delta_x),
                                            (j*(MAT_WIDTH/NUMOFTAXELS_Y) -
                                                MAT_HALF_WIDTH),
                                            self.bed_height + delta_z]
                taxel_count = taxel_count + 1
         
        return surface_grid


    def run (self):
        rate = rospy.Rate(30) #30 Hz

        joint_state = JointState()

        dict_of_links = ({'/head_rest_link':0.762659,
                          '/leg_rest_upper_link':1.04266,
                          '/leg_rest_lower_link':1.41236})
        list_of_links = dict_of_links.keys()
        while not rospy.is_shutdown():
            joint_state.header.stamp = rospy.Time.now()
            #Resize the pressure map data
            p_map = np.reshape(self.pressuremap_flat, (NUMOFTAXELS_X,
                NUMOFTAXELS_Y))
            #Clear pressure map grid
            #Filter data
            self.filter_data()
            joint_state.name = [None]*(9)
            joint_state.position = [None]*(9)
            joint_state.name[0] = "autobed_height_joint"
            joint_state.name[1] = "head_rest_hinge"
            joint_state.name[2] = "leg_rest_upper_joint"
            joint_state.name[3] = "leg_rest_upper_lower_joint"
            joint_state.name[4] = "X"
            joint_state.name[5] = "mid_body_support"
            joint_state.name[6] = "head_support"
            joint_state.name[7] = "leg_rest_lower_support"
            joint_state.name[8] = "leg_rest_upper_support"
            joint_state.position[0] = self.bed_height
            joint_state.position[1] = self.head_filt_data
            joint_state.position[2] = self.leg_filt_data
            joint_state.position[3] = -(1+(4.0/9.0))*self.leg_filt_data
            joint_state.position[4] = 0
            joint_state.position[5] = 0
            joint_state.position[6] = 0
            joint_state.position[7] = 0
            joint_state.position[8] = 0
            self.joint_pub.publish(joint_state)

            '''Send Kinect pose to TF, so that we can get the exact pose of the
            camera'''
            self.sendToRviz.sendTransform(self.camera_p,
                                          self.camera_q,
                                          rospy.Time.now(),
                                          "camera_link",
                                          "world")
            #Sample the bed surface into a grid depicting the pressure mat
            surface_grid = self.sample_bed_surface()
            #Transfer parts of the grid to each link frame 
            #TODO: MAKE THIS CODE CLEANER
            alpha = self.head_filt_data#Just some constants
            beta = self.leg_filt_data
            leg_length = (dict_of_links['/leg_rest_lower_link'] -
                            dict_of_links['/leg_rest_upper_link'])
            #Compute angles of the pressure map grid
            for i in range(NUMOFTAXELS_X*NUMOFTAXELS_Y):
                if surface_grid[i][0] < dict_of_links['/head_rest_link']:
                    r = dict_of_links['/head_rest_link'] - surface_grid[i][0]
                    surface_grid[i][0] += (r - r*cos(alpha))
                    surface_grid[i][2] += r*sin(alpha)
                elif (surface_grid[i][0] < (
                    dict_of_links['/leg_rest_upper_link']+leg_length*cos(beta))
                    and 
                    surface_grid[i][0] > dict_of_links['/leg_rest_upper_link']):
                    r = (surface_grid[i][0] - 
                            dict_of_links['/leg_rest_upper_link'])
                    surface_grid[i][0] -= (r - r*cos(beta))
                    surface_grid[i][2] += r*sin(beta)
                elif surface_grid[i][0] > (
                    dict_of_links['/leg_rest_upper_link']+leg_length*cos(beta)):
                    surface_grid[i][2] += leg_length*sin(beta)

            #Plotting the pressure map on the mat surface as a CUBE LIST
            marker = Marker()
            marker.header.frame_id = '/world'
            marker.type = marker.POINTS
            marker.action = marker.ADD
            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 1.0
            marker.pose.orientation.w = 1.0
            for i in range(NUMOFTAXELS_X*NUMOFTAXELS_Y):
                cell = Point()
                cell_color = ColorRGBA()
                cell.x = surface_grid[i][0]
                cell.y = surface_grid[i][1]
                cell.z = surface_grid[i][2]
                if self.pressuremap_flat[0][i] > 0:
                    cell_color.r = 1.0
                    cell_color.g = (100.0 - self.pressuremap_flat[0][i])/100.0
                    cell_color.b = 0.0
                    cell_color.a = 1.0
                else:
                    cell_color.r = 0.0
                    cell_color.g = 0.0
                    cell_color.b = 1.0
                    cell_color.a = 1.0
                   
                marker.points.append(cell)
                marker.colors.append(cell_color)

            self.pressure_grid_pub.publish(marker)
            rate.sleep()
        return  


if __name__ == "__main__":

    a=AutobedConverter()
    a.run()
