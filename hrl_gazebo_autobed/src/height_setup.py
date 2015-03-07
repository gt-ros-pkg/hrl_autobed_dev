#!/usr/bin/env python

import roslib; roslib.load_manifest('hrl_gazebo_autobed')
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pr2_controllers_msgs.msg import JointTrajectoryControllerState

class JointTrajectoryTest():
    def __init__(self, controller='/autobed_height_controller'):
        self.controller = controller
        self.goal_pub = rospy.Publisher(controller+'/command', JointTrajectory)
        self.state_sub = rospy.Subscriber(controller+'/state', 
                JointTrajectoryControllerState, self.state_cb)
        self.joint_names = None

    def state_cb(self, state_msg):
        if self.joint_names is None:
            self.joint_names = state_msg.joint_names

    def up_msg(self, angle):
        jtm = JointTrajectory()
        jtm.joint_names = self.joint_names
        jtp = JointTrajectoryPoint()
        jtp.positions = [angle]*len(self.joint_names)
        jtp.time_from_start = rospy.Duration(1.0)
        jtm.points = [jtp]
        return jtm

    def run(self, angle):
        while self.joint_names is None:
            print "Waiting for joint state information from %s/state topic" %self.controller
            rospy.sleep(2)
        print "Received joint state information. Sending bed to default position"
        up_msg = self.up_msg(angle)
        self.goal_pub.publish(up_msg)
        return

if __name__=='__main__':
    rospy.init_node('autobed_height_setup')
    JTT = JointTrajectoryTest()
    JTT_h = JointTrajectoryTest(controller='/autobed_head_controller')
    JTT_l = JointTrajectoryTest(controller='/autobed_legs_controller')
    JTT_passive = JointTrajectoryTest(controller='/autobed_passive_joint_controller')
    JTT.run(0.3)
    JTT_h.run(0.0)
    JTT_passive.run(-0.3)
    JTT_l.run(0.3)
