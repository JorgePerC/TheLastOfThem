#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import tf

class Odometry:
        # Static variables
    PI = 3.1416
    g = 9.81

    def __init__(self, robot_r, robot_h, robot_d, tmsInSec = 25):
        rospy.init_node("odometry")
        rospy.loginfo("Starting ROSNode as odometry.")

        # Subscribers:
        self.sub_wl = rospy.Subscriber("/robot/wl", Float32, self.get_wl)
        self.sub_wr = rospy.Subscriber("/robot/wr", Float32, self.get_wr)

        # Publishers:
        self.pub_pose = rospy.Publisher("/robot/pose", Pose2D, queue_size=10)
        self.pubish_tf  = tf.TransformBroadcaster()
        
        # Sensor vector
        # * wl
        # * wr
        # * 0, but needed for multiplication
        self.sensorVect = np.array([[0.0, 0.0, 0.0]]).T

        # Integration for robot pose
            # We asume 0 as start
            # This is actually odom
        self.pose = Pose2D()
        self.pose.x = 0
        self.pose.y = 0
        self.pose.theta = Odometry.PI/2
        
        # Robot characteristics:
        self.r = robot_r
        self.d = robot_d
        self.h = robot_h
        
        self.dt = 1.0/tmsInSec
        print(self.dt)
        self.rate = rospy.Rate(tmsInSec)

        
        rospy.on_shutdown(self.stop)
        

    def get_wl(self, msg):
        self.sensorVect[0, 0] = msg.data 
        
    def get_wr(self, msg):
        self.sensorVect[1, 0] = msg.data

    def stop(self):
        rospy.loginfo("Ended puzzlebot")
    
    def runrum(self):
        # Update angular velocity from the wl and wr
            # Haha I lied, we can't measure that state
            # We are using the dMatrix to get from only 2 readings the 
            # 3 sized state vector
        
        # Transformation matrix 
        dMatrix = np.array([
            [self.r/2*np.cos( self.pose.theta )-self.h*self.r/self.d*np.sin( self.pose.theta ), 
                self.r/2*np.cos( self.pose.theta )+self.h*self.r/self.d*np.sin( self.pose.theta ), 0],

            [self.r/2*np.sin( self.pose.theta )-self.h*self.r/self.d*np.cos( self.pose.theta ), 
                self.r/2*np.sin( self.pose.theta )+self.h*self.r/self.d*np.cos( self.pose.theta ), 0],

            [self.r/self.d, 
                -self.r/self.d, 0]])

        # Calculate the derivative on this itegration
        thisIteration = np.matmul(dMatrix, self.sensorVect)

        # Update the actual stimation
            # Ideally, this should be the same as the time we run the program on the STM32
        self.pose.x = self.pose.x + thisIteration[0, 0] * self.dt
        self.pose.y = self.pose.y + thisIteration[1, 0] * self.dt
        self.pose.theta = self.pose.theta + thisIteration[2, 0] *self.dt
        #print(self.dt )
        # print("x: ", self.pose.x, "y: ", self.pose.y )
        # self.pubish_tf.sendTransform((msg.x, msg.y, 0),
        #     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
        #     rospy.Time.now(),
        #     turtlename,
        #     "world")
        
        self.pub_pose.publish(self.pose)
    
if __name__ == "__main__":
    odom = Odometry(0.048, 0.1, 0.3)
    while not rospy.is_shutdown():
        odom.runrum()
        odom.rate.sleep()