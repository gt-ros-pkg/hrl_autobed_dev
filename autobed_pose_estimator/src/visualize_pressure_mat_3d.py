import numpy as np
import roslib; roslib.load_manifest('hrl_msgs')
import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from hrl_msgs.msg import FloatArrayBare 


class Visualize3D():

    def __init__(self):

        rospy.Subscriber("/fsascan", FloatArrayBare, self.pressure_map_callback)
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        a=np.linspace(0, 3.14, 64)
        b=np.linspace(0, 3.14, 27)
        self.physical_pressure_map=np.random.rand(64, 27)
    def pressure_map_callback(self, data):
        '''This callback accepts incoming pressure map from 
        the Vista Medical Pressure Mat and sends it out. 
        Remember, this array needs to be binarized to be used'''
        self.physical_pressure_map = np.resize(np.asarray(data.data), tuple(64, 27))

    def run(self):
        x, y=np.meshgrid(np.linspace(0, 63, 64), np.linspace(0, 26, 27));
        z=self.physical_pressure_map
        
        self.ax.plot_surface(x, y, z,  rstride=4, cstride=4, color='b')
        plt.show()
        


if __name__ == "__main__":
    a=Visualize3D()
    rospy.init_node('visualize_pressure_3D', anonymous=True) 
    rate=rospy.Rate(5)
    while not rospy.is_shutdown():
        a.run()
        rate.sleep()

