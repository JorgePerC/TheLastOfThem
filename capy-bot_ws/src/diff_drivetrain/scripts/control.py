#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry

class PoseControl:
    def __init__(self, threshold , repsInSec = 25):
        rospy.init_node("poseControl")
        rospy.loginfo("Starting PoseControl as control")
        
        # ===== Subscribers =====
        self.sub_pose = rospy.Subscriber("/robot/kalmanPose", Odometry, self.get_poseRobot)
        self.sub_poseD = rospy.Subscriber("/robot/objective", Pose2D, self.get_poseDeseada)

        # ===== Publishers =====
        self.pub_wl = rospy.Publisher("/robot/set_wl", Float32, queue_size=5)
        self.pub_wr = rospy.Publisher("/robot/set_wr", Float32, queue_size=5)
        
        # # ===== Rate =====
        self.rate = rospy.Rate(repsInSec)
        self.dt = 1.0/repsInSec

        # ===== Shutdown =====
        rospy.on_shutdown(self.stop)

        # ===== Params =====
        self.r = rospy.get_param("/Capybot/wheelRadius")
        self.d = rospy.get_param("/Capybot/dPoint")
        self.h = rospy.get_param("/Capybot/wheelDistance")
        
        # ===== Class attributes =====
        # Robot characteristics
            # X
            # Y
            # Theta
        self.sensorVect = np.array([[0.0, 0.0, 0.0]]).T
        # Set point
            # X   
            # Y
            # Theta 
        self.q_deseada = np.array([[0.0, 0.0]]).T
        self.threshold = threshold

    def run_control(self):
        # Transformation matrix 

        dMatrix = np.array([
            [self.r*np.cos(self.sensorVect[2,0])/2 - self.h*self.r*np.sin(self.sensorVect[2,0])/self.d, 
                self.r*np.cos(self.sensorVect[2,0])/2 + self.h*self.r*np.sin(self.sensorVect[2,0])/self.d],

            [self.r*np.sin(self.sensorVect[2,0])/2 + self.h*self.r*np.cos(self.sensorVect[2,0])/self.d, 
                self.r*np.sin(self.sensorVect[2,0])/2 - self.h*self.r*np.cos(self.sensorVect[2,0])/self.d]])

        K = np.array([  [0.12, 0.0], 
                        [0.0, 0.24]])

        # Calculate error
        estado = np.array([[self.sensorVect[0,0], self.sensorVect[1,0]]]).T
        error = self.q_deseada - estado 

        # Stop if we are near the objective

        if self.isRobotClose():
            # print("----------LLEGO-------")
            
            self.pub_wr.publish(0.0)
            self.pub_wl.publish(0.0)
            return
        # Calculate control 
        u = np.matmul(np.linalg.inv(dMatrix),
                        #self.q_deseada_punto + # We ignore this bc: we don't want to finish in an specific velocity
                        np.dot(K, error))
        # Limit control wr
        if (u[0, 0] < -8):
            u[0, 0] = -8
        elif (u[0, 0] > 8):
            u[0, 0] = 8
        # Limit control wl
        if (u[1, 0] < -8):
            u[1, 0] = -8
        elif (u[1, 0] > 8):
            u[1, 0] = 8

        # Min vel wr
        if (0.2 < u[0, 0] < 1.5):
            u[0, 0] = 1.5
        elif (-1.5 < u[0, 0] < -0.2):
            u[0, 0] = -1.5
        
        # Min vel wl
        if (0.2 < u[1, 0] < 1.5):
            u[1, 0] = 1.5
        elif (-1.5 < u[1, 0] < -0.2):
            u[1, 0] = -1.5

        self.pub_wr.publish(u[0, 0])
        self.pub_wl.publish(u[1, 0])
        
    def get_poseDeseada(self, msg):
        self.q_deseada[0, 0] = msg.x
        self.q_deseada[1, 0] = msg.y 

    def get_poseRobot(self, msg):
        self.sensorVect[0, 0] = msg.pose.pose.position.x
        self.sensorVect[1, 0] = msg.pose.pose.position.y 
        self.sensorVect[2, 0] = msg.pose.pose.orientation.w 
    
    def isRobotClose(self):
        x = self.q_deseada[0,0] - self.threshold < self.sensorVect[0,0] <  self.q_deseada[0,0] + self.threshold
        y = self.q_deseada[1,0] - self.threshold < self.sensorVect[1,0] <  self.q_deseada[1,0] + self.threshold
        return x and y

    def stop(self):
        # Set deseada as actual
        self.q_deseada = self.sensorVect.copy()
        rospy.loginfo("Ended control")

    
if __name__ == "__main__":
        # 3 cm threshold
    control = PoseControl(threshold= 0.05, repsInSec = 40)
    while not rospy.is_shutdown():
        control.run_control()
        control.rate.sleep()
