#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D

class PoseControl:
    def __init__(self, robot_r, robot_d, robot_h, threshold , repsInSec = 25):
        rospy.init_node("poseControl")
        rospy.loginfo("Starting PoseControl as control")
        # Subscribers:
        self.sub_pose = rospy.Subscriber("/robot/pose", Pose2D, self.get_poseRobot)
        self.sub_poseD = rospy.Subscriber("/robot/objective", Pose2D, self.get_poseDeseada)

        # Publishers:
        self.pub_wl = rospy.Publisher("/robot/set_wl", Float32, queue_size=5)
        self.pub_wr = rospy.Publisher("/robot/set_wr", Float32, queue_size=5)
        
        # Rate
        self.rate = rospy.Rate(repsInSec)
        self.dt = 1.0/repsInSec

        # On shutdown
        rospy.on_shutdown(self.stop)

        # Robot characteristics:
        self.r = robot_r
        self.d = robot_d
        self.h = robot_h
        
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
        """
        dMatrix = np.array([
            [self.r/2*np.cos( self.sensorVect[2, 0] )-self.h*self.r/self.d*np.sin( self.sensorVect[2, 0] ), 
                self.r/2*np.cos( self.sensorVect[2, 0] )+self.h*self.r/self.d*np.sin( self.sensorVect[2, 0] ), 0],

            [self.r/2*np.sin( self.sensorVect[2, 0] )-self.h*self.r/self.d*np.cos( self.sensorVect[2, 0] ), 
                self.r/2*np.sin( self.sensorVect[2, 0] )+self.h*self.r/self.d*np.cos( self.sensorVect[2, 0] ), 0],

            [self.r/self.d, 
                -self.r/self.d, 0]])
        """
        dMatrix = np.array([
            [self.r/2*np.cos( self.sensorVect[2, 0] ) - self.h*self.r/self.d*np.sin( self.sensorVect[2, 0] ), 
                self.r/2*np.cos( self.sensorVect[2, 0] ) + self.h*self.r/self.d*np.sin( self.sensorVect[2, 0] )],

            [self.r/2*np.sin( self.sensorVect[2, 0] ) - self.h*self.r/self.d*np.cos( self.sensorVect[2, 0] ), 
                self.r/2*np.sin( self.sensorVect[2, 0] ) + self.h*self.r/self.d*np.cos( self.sensorVect[2, 0] )]])

        K = np.array([[1.0, 0.0], 
                        [0.0, 2.0]])

        # Calculate error
        estado = np.array([[self.sensorVect[0,0], self.sensorVect[1,0]]]).T
        error = self.q_deseada - estado 

        # Stop if we are near the objective
        if np.average(error) < self.threshold:
            error = 0

        # Calculate control 
        u = np.matmul( np.linalg.inv(dMatrix),
                        estado + 
                            np.dot(K, error) )
        # Send control
        self.pub_wr.publish(u[0, 0])
        self.pub_wl.publish(u[1, 0])
        
    def get_poseDeseada(self, msg):
        self.q_deseada[0, 0] = msg.x
        self.q_deseada[1, 0] = msg.y 
        #self.q_deseada[2, 0] = msg.theta 

    def get_poseRobot(self, msg):
        self.sensorVect[0, 0] = msg.x
        self.sensorVect[1, 0] = msg.y 
        self.sensorVect[2, 0] = msg.theta 
        

    def stop(self):
        # Set deseada as actual
        self.q_deseada = self.sensorVect.copy()
        rospy.loginfo("Ended control")

    
if __name__ == "__main__":
        # 3 cm threshold
    control = PoseControl(0.048, 0.1, 0.3, threshold= 0.03)
    while not rospy.is_shutdown():
        control.run_control()
        control.rate.sleep()