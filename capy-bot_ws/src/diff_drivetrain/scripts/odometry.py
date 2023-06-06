#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D


class OdometryRover:
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
        self.pub_pose = rospy.Publisher("/robot/noisyOdom", Odometry, queue_size=5)
        
        # ===== Params =====
        self.r = rospy.get_param("/Capybot/wheelRadius")
        self.d = rospy.get_param("/Capybot/dPoint")
        self.h = 0

        # ===== Class attributes =====
        # Sensor vector
            # wl
            # wr
            # 0, but needed for multiplication
        self.sensorVect = np.array([[0.0, 0.0]]).T

        self.u = np.array([[0.0, 0.0]]).T

        # Integration for robot pose
            # We asume 0 as start
            # This is actually odom
        self.p = Odometry()
        self.p.header.frame_id = "world"
        self.p.pose.pose.position.x = 0
        self.p.pose.pose.position.y = 0
        self.p.pose.pose.orientation.w = 0 #Odometry.PI/2 TODO: Try alternative


        # To adjust our angle:
        self.adj = np.array([[1.0, 1.0, 1.0]]).T
        self.angVel = 0
        # ===== Rate =====
        self.rate = rospy.Rate(tmsInSec)
        self.dt = 1.0/tmsInSec

        # ===== Shutdown =====
        rospy.on_shutdown(self.stop)
        
    def get_wr(self, msg):
        self.sensorVect[0, 0] = msg.data 
        
    def get_wl(self, msg):
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
            [self.r*np.cos(self.p.pose.pose.orientation.w)/2 - self.h*self.r*np.sin(self.p.pose.pose.orientation.w)/self.d, 
                self.r*np.cos(self.p.pose.pose.orientation.w)/2 + self.h*self.r*np.sin(self.p.pose.pose.orientation.w)/self.d],

            [self.r*np.sin(self.p.pose.pose.orientation.w)/2 + self.h*self.r*np.cos(self.p.pose.pose.orientation.w)/self.d, 
                self.r*np.sin(self.p.pose.pose.orientation.w)/2 - self.h*self.r*np.cos(self.p.pose.pose.orientation.w)/self.d],

            [self.r/self.d, 
                -self.r/self.d]])

        # Calculate the derivative on this itegration
        thisIteration = np.matmul(dMatrix, self.sensorVect) 

        # Adjust angle multiplying it by two

        # Update the actual stimation
            # Ideally, this should be the same as the time we run the program on the STM32
        self.p.pose.pose.position.x = self.p.pose.pose.position.x + thisIteration[0, 0] * self.dt
        self.p.pose.pose.position.y = self.p.pose.pose.position.y + thisIteration[1, 0] * self.dt
        self.p.pose.pose.orientation.w = self.p.pose.pose.orientation.w + thisIteration[2, 0] *self.dt

        # Limit pose angle
        # self.p.pose.pose.orientation.w = self.p.pose.pose.orientation.w%(2*OdometryRover.PI)
        # Update time
        self.p.header.stamp = rospy.Time.now()

        # Send pose
        self.pub_pose.publish(self.p)
    
if __name__ == "__main__":
    odom = OdometryRover()
    while not rospy.is_shutdown():
        odom.runrum()
        odom.rate.sleep()
