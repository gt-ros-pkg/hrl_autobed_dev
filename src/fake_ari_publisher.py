#!/usr/bin/env python
# license removed for brevity
import rospy
import roslib; roslib.load_manifest('hrl_autobed_dev')

from std_msgs.msg import Float32
from hrl_msgs.msg import FloatArrayBare
def talker():
    pub = rospy.Publisher('/abdin0', FloatArrayBare)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 0.01Hz , i.e 100 secs
    while not rospy.is_shutdown():
        msg = [0.0, 20.0, 0.0]
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

if __name__=="__main__":
    talker()

                                                                 
                                                                 
