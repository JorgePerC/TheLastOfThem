#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D


def euler2quaternion(e):
    # Euler should be formated as
        # yaw
        # pitch
        # roll
    res = [0, 0, 0, 0]

    cy = np.cos(e[0] * 0.5)
    sy = np.sin(e[0] * 0.5)

    res[0] = cy         # w
    res[1] = 0.0        # x
    res[2] = 0.0        # y
    res[3] = sy         # z
    return res

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

        self.P = np.array([[0.0, 0.0, 0.0]]).T

        self.u = np.array([[0.0, 0.0]]).T

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
            [self.r*np.cos(self.P[2,0])/2 - self.h*self.r*np.sin(self.P[2,0])/self.d, 
                self.r*np.cos(self.P[2,0])/2 + self.h*self.r*np.sin(self.P[2,0])/self.d],

            [self.r*np.sin(self.P[2,0])/2 + self.h*self.r*np.cos(self.P[2,0])/self.d, 
                self.r*np.sin(self.P[2,0])/2 - self.h*self.r*np.cos(self.P[2,0])/self.d],

            [self.r/self.d, 
                -self.r/self.d]])

        # Calculate the derivative on this itegration
        thisIteration = np.matmul(dMatrix, self.sensorVect) 

        # Adjust angle multiplying it by two

        # Update the actual stimation
            # Ideally, this should be the same as the time we run the program on the STM32
        self.P[0,0] = self.P[0,0] + thisIteration[0, 0] * self.dt
        self.P[1,0] = self.P[1,0] + thisIteration[1, 0] * self.dt
        self.P[2,0] = self.P[2,0] + thisIteration[2, 0] * self.dt

        msg = Odometry()
        msg.header.frame_id = "world"
        msg.pose.pose.position.x = self.P[0, 0]
        msg.pose.pose.position.y = self.P[1, 0]

        q = euler2quaternion([self.P[2, 0], 0.0, 0.0])

        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3] #Odometry.PI/2 TODO: Try alternative

        # Limit pose angle
        # self.p.pose.pose.orientation.w = self.p.pose.pose.orientation.w%(2*OdometryRover.PI)
        # Update time
        msg.header.stamp = rospy.Time.now()

        # Send pose
        self.pub_pose.publish(msg)
    
if __name__ == "__main__":
    odom = OdometryRover()
    while not rospy.is_shutdown():
        odom.runrum()
        odom.rate.sleep()
