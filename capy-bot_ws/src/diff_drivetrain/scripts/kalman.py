#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import tf

class KalmanOdometry:
    def __init__(self, robot_r, robot_h, robot_d, robot_m = 5, repsInSec = 25):
        rospy.init_node("Kalman Odometry")
        rospy.loginfo("Starting ROSNode as Kalman odometry.")
        # Subscribers:
        self.sub_wl = rospy.Subscriber("/robot/wl", Float32, self.get_wl)
        self.sub_wr = rospy.Subscriber("/robot/wr", Float32, self.get_wr)

        # Publishers:
        self.pub_pose = rospy.Publisher("/robot/pose", Pose2D, queue_size=10)
        self.pubish_tf  = tf.TransformBroadcaster()
        
        # Rate
        self.rate = rospy.Rate(repsInSec)
        self.dt = 1.0/repsInSec
        
        # On shutdown
        rospy.on_shutdown(self.stop)

        # Robot characteristics
        self.r = robot_r
        self.m = robot_m
        self.h = robot_h
        self.d = robot_d
        # I = 1        

        # Robot states
        self.q_deseada = np.array([[0.0,  0.0,  0.0]]).T

        # Aka. q (estado inicial)
        self.xP = np.array([[0.0, 0.0, 0.0]]).T

        self.K = np.array([[1, 0, 0], 
                            [0, 2, 0], 
                            [0, 0, 0]])

        # Aka z
            # In this case our sensors are exactly the dynamics
        # Sensor vector
        # * wl
        # * wr
        # * 0, but needed for multiplication
        self.sensorVect = np.array([[0.0, 0.0, 0.0]]).T

        # Aka H
            # Identity-ish matrix for sensor input
            # Vector space from sensor readouts to statesX
            # Virtual sensor obtained from odometry
                # x
                # y
                # theta
        self.statesSensor = np.array([[1.0, 0.0, 0.0], 
                                        [0.0, 1.0, 0.0],
                                        [0.0, 0.0, 1.0]]) 

        # Aka Q
            # Covariance from model noise
        self.model_Cov_Mat = np.array([ [1.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0],
                                [0.0, 0.0, 1.0]]) 

        # Aka R
            # Covariance from sensor input
        self.sensor_Cov_Mat = np.array([ [1.0, 0.0, 0.0], 
                                    [0.0, 1.0, 0.0],
                                    [0.0, 0.0, 1.0]]) 

        # Aka P
            # Covariance from state estimation 
        self.prediction_Cov_Mat = np.array([ [0.0, 0.0, 0.0], 
                                        [0.0, 0.0, 0.0],
                                        [0.0, 0.0, 0.0]])
        


    def runKalman(self):
    
        # Calculate error
        error = self.q_deseada - self.xP 
        
        # Calculate control (actuator force)
        u = np.matmul(self.K, error)

        # System dynamics
        A = np.array([[self.r/2*np.cos( self.xP[2,0] )-self.h*self.r/self.d*np.sin( self.xP[2,0] ), self.r/2*np.cos( self.xP[2,0] )+self.h*self.r/self.d*np.sin( self.xP[2,0] ), 0],
                [self.r/2*np.sin( self.xP[2,0] )-self.h*self.r/self.d*np.cos( self.xP[2,0] ), self.r/2*np.sin( self.xP[2,0] )+self.h*self.r/self.d*np.cos( self.xP[2,0] ), 0],
                [self.r/self.d, -self.r/self.d, 0]])

        # Update sensor readout
        # sensorVect = self.sensorVect 
        # It's done on the callbacks

        # Update prediction 
        self.xP = self.xP + self.dt*( 
            np.matmul(A, self.xP) + u ) 
        """+
            # Kalman filter stuff
            np.matmul(self.prediction_Cov_Mat,
                np.matmul(self.statesSensor.T,
                        np.matmul(np.linalg.inv(self.sensor_Cov_Mat),
                            np.matmul((self.sensorVect-self.statesSensor,
                                self.xP))))))   
        

        # Update prediction covariance matrix
        self.prediction_Cov_Mat = self.prediction_Cov_Mat + self.dt*(
                np.matmul(A, self.prediction_Cov_Mat) 
                    + np.matmul(self.prediction_Cov_Mat, A.T) 
                    + self.model_Cov_Mat -
                        np.matmul(self.prediction_Cov_Mat,
                        np.matmul(self.statesSensor.T,
                            np.matmul(
                                np.linalg.inv(self.sensor_Cov_Mat),
                                np.matmul(self.statesSensor,
                                        self.prediction_Cov_Mat)))))
        """
    def get_wl(self, msg):
        self.sensorVect[0, 0] = msg.data 
        
    def get_wr(self, msg):
        self.sensorVect[1, 0] = -msg.data

    def stop(self):
        rospy.loginfo("Ended puzzlebot")

if __name__ == "__main__":
    kOdom = KalmanOdometry()
    while not rospy.is_shutdown():
        kOdom.runKalman()
        kOdom.rate.sleep()