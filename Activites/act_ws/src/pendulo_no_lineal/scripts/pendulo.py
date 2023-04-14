#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D

# Matrix libs
import numpy as np

class PenduloNoLineal:
    # Static variables
    PI = 3.1416
    g = 9.81

    def __init__(self, tao, initX = PI/4, initY = 0):
        rospy.init_node("penduloNoLineal")
        rospy.loginfo("Starting penduloNoLineal as name_node.")
        
        self.message_pub = rospy.Publisher("pose", Pose2D, queue_size=5)
        self.rate = rospy.Rate(1/tao)
        self.tao = tao
        rospy.on_shutdown(self.stop)

        # Our states are:
        # * PositionX
        # * PositionY
        self.states = np.array([[initX, initY]]).T

        self.A = np.array([[0, 1], [-PenduloNoLineal.g, 0]])
        self.B = np.array([[0, 1]]).T

    def simulate (self):
        # Error matrix
        u = 0     #Control with e
        
        self.states = self.states + self.tao*(np.matmul(self.A, self.states) + self.B*u)

        msg = Pose2D()
        msg.x = self.states[0]
        msg.y = self.states[1]

        self.message_pub.publish(msg)
    
    
    def stop(self):
        rospy.loginfo("Stopping")


if __name__ == '__main__':
    
    pp = PenduloNoLineal(0.02)
    while not rospy.is_shutdown():
        pp.simulate()
        pp.rate.sleep()
    

    