#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D


class Odometry:
        # Static variables
    PI = 3.1416
    g = 9.81

    def __init__(self, tmsInSec = 40):
        rospy.init_node("odometry")
        # ===== Wait for STM32 ===== 
        try:
            rospy.loginfo("Waiting for encoder vels from Nucleo board")
            rospy.wait_for_message('/robot/wr', Float32, timeout = 5)
            rospy.wait_for_message('/robot/wl', Float32, timeout = 5)
        except rospy.ROSException as e:
            print("Velocities not found")
            rospy.logerr("Can't start odometry. No wl/wr")
            rospy.logerr(e)

        rospy.loginfo("Starting ROSNode as odometry.")
        
        # ===== Subscribers =====
        self.sub_wl = rospy.Subscriber("/robot/wl", Float32, self.get_wl)
        self.sub_wr = rospy.Subscriber("/robot/wr", Float32, self.get_wr)

        # ===== Publishers =====
        self.pub_pose = rospy.Publisher("/robot/pose", Pose2D, queue_size=10)
        
        # ===== Params =====
        self.r = rospy.get_param("/Capybot/wheelRadius")
        self.d = rospy.get_param("/Capybot/dPoint")
        self.h = 0

        # ===== Class attributes =====
        # Sensor vector
            # wl
            # wr
            # 0, but needed for multiplication
        self.sensorVect = np.array([[0.0, 0.0, 0.0]]).T

        self.u = np.array([[0.0, 0.0, 0.0]]).T

        # Integration for robot pose
            # We asume 0 as start
            # This is actually odom
        self.pose = Pose2D()
        self.pose.x = 0
        self.pose.y = 0
        self.pose.theta = 0 #Odometry.PI/2 TODO: Try alternative

        # To adjust our angle:
        self.adj = np.array([[1.0, 1.0, 1.0]]).T
        self.angVel = 0
        # ===== Rate =====
        self.rate = rospy.Rate(tmsInSec)
        self.dt = 1.0/tmsInSec

        # ===== Shutdown =====
        rospy.on_shutdown(self.stop)
        

    def get_wl(self, msg):
        self.sensorVect[0, 0] = msg.data 
        
    def get_wr(self, msg):
        self.sensorVect[1, 0] = msg.data
    
    def stop(self):
        rospy.loginfo("Ended odometry")
    
    def runrum(self):
        # Update angular velocity from the wl and wr
            # Haha I lied, we can't measure that state
            # We are using the dMatrix to get from only 2 readings the 
            # 3 sized state vector
        
        # Transformation matrix 
        dMatrix = np.array([
            [self.r*np.cos(self.pose.theta)/2 - self.h*self.r*np.sin(self.pose.theta)/self.d, 
                self.r*np.cos(self.pose.theta)/2 + self.h*self.r*np.sin(self.pose.theta)/self.d, 0],

            [self.r*np.sin(self.pose.theta)/2 + self.h*self.r*np.cos(self.pose.theta)/self.d, 
                self.r*np.sin(self.pose.theta)/2 - self.h*self.r*np.cos(self.pose.theta)/self.d, 0],

            [self.r/self.d, 
                -self.r/self.d, 0]])
        
        ### Trying something new
        #F = np.array([[1.0, 0, 0],
        #              [0, 1.0, 0],
        #              [0, 0, 1.0]])
        
        #thisIteration = np.dot(F, self.sensorVect) + np.dot(dMatrix, self.u)

        # Calculate the derivative on this itegration
        thisIteration = np.matmul(dMatrix, self.sensorVect) #*self.adj

        # Adjust angle multiplying it by two

        # Update the actual stimation
            # Ideally, this should be the same as the time we run the program on the STM32
        self.pose.x = self.pose.x + thisIteration[0, 0] * self.dt
        self.pose.y = self.pose.y + thisIteration[1, 0] * self.dt
        self.pose.theta = self.pose.theta + thisIteration[2, 0] *self.dt

        # Limit theta angle
        self.pose.theta = self.pose.theta%2*Odometry.PI
        # if (self.pose.theta > Odometry.PI):
        #     self.pose.theta -= 2*Odometry.PI
        # elif (self.pose.theta < -Odometry.PI):
        #     self.pose.theta += 2*Odometry.PI
        
        self.pub_pose.publish(self.pose)
    
if __name__ == "__main__":
    odom = Odometry()
    while not rospy.is_shutdown():
        odom.runrum()
        odom.rate.sleep()
