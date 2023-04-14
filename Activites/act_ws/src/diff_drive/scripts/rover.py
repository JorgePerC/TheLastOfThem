#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D

# Matrix libs
import numpy as np

class Puzzlebotis:
    # Static variables
    PI = 3.1416
    g = 9.81

    def __init__(self, tao = 0.01, r = 0.07, m = 1, l = 0.2, I = 1, initX = PI/4, initY = 0):
        rospy.init_node("puzzlebot")
        rospy.loginfo("Starting node as puzzlebot.")
        
        self.message_pub = rospy.Publisher("position", Pose2D, queue_size=5)
        self.rate = rospy.Rate(1/tao)
        self.tao = tao
        # r -> robot
        self.r = {"r": r,
                    "m": m,
                    "l": l,
                    "I": I}
        rospy.on_shutdown(self.stop)
        
        # This control is
        # * x
        # * y
        # * theta_d
        # * vel_d
        # * omega_d
        self.states = np.array([[0.0, 0.0, 0.0, 0.0, 0.0]]).T

        A = np.array([[0, 1],  [-Puzzlebotis.g, 0]])
        B = np.array([[0, 1]]).T

    def stop(self):
        rospy.loginfo("Ended puzzlebot")
    
    def runrum(self):
        u = np.array([[1,0]]).T
        xdot = self.states[3]*np.cos(self.states[2])
        ydot = self.states[3]*np.sin(self.states[2])
        tdot = self.states[4]
        vdot = (u[0] + u[1])/(self.r["r"]*self.r["m"])
        odot = self.r["l"]*(u[0]-u[1])/(self.r["r"]*self.r["I"])

        self.states[0] = self.states[0] + self.tao*(xdot)
        self.states[1] = self.states[1] + self.tao*(ydot)
        self.states[2] = self.states[2] + self.tao*(tdot)
        self.states[3] = self.states[3] + self.tao*(vdot)
        self.states[4] = self.states[4] + self.tao*(odot)

        msg = Pose2D()
        msg.x = self.states[0]
        msg.y = self.states[1]
        msg.theta = self.states[2]

        self.message_pub.publish(msg)

if __name__ == '__main__':
    
    pp = Puzzlebotis(0.02)
    while not rospy.is_shutdown():
        pp.runrum()
        pp.rate.sleep()
    