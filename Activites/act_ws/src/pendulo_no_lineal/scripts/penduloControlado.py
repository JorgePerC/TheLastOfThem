#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D

# Matrix libs
import numpy as np

class PenduloControlado:
    # Static variables
    PI = 3.1416
    g = 9.81

    def __init__(self, tao, initX = PI/4, initY = 0):
        rospy.init_node("penduloControlado")
        rospy.loginfo("Starting penduloControlado as name_node.")
        
        self.message_pub = rospy.Publisher("pose", Pose2D, queue_size=5)
        self.rate = rospy.Rate(1/tao)
        self.tao = tao
        rospy.on_shutdown(self.stop)

        # Our states are:
        # * PositionX
        # * PositionY
        self.states = np.array([[initX, initY]]).T

    def simulate (self):
        # Error matrix
        e = self.states - np.array([ [PenduloControlado.PI/3,0] ]).T  
        u = -190.2*e[0] - 30*e[1]     #Control with e

        xdot_0 = self.states[1]
        xdot_1 = -PenduloControlado.g*np.sin(self.states[0]) + u
        
        #x = x + self.tao*(A@x + B*u)
        self.states[0] = self.states[0] + self.tao * xdot_0
        self.states[1] = self.states[1] + self.tao * xdot_1
        
        msg = Pose2D()
        msg.x = self.states[0]
        msg.y = self.states[1]

        self.message_pub.publish(msg)
    
    
    def stop(self):
        rospy.loginfo("Stopping")


if __name__ == '__main__':
    
    pp = PenduloControlado(0.02)
    while not rospy.is_shutdown():
        pp.simulate()
        pp.rate.sleep()
    

    