#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import tf

class KalmanOdometry:
    def __init__(self, odomFreq, repsInSec = 25):
        rospy.init_node("Kalman Odometry")
        rospy.loginfo("Starting ROSNode as Kalman odometry.")
        
        # ===== Subscribers =====
        self.sub_pose = rospy.Subscriber("/robot/pose", Pose2D, self.get_pose)

        # ===== Publishers =====
        self.pub_poseK = rospy.Publisher("/robot/kalmanPose", Pose2D, queue_size=10)
        self.pubish_tf  = tf.TransformBroadcaster()
        
        # ===== Params =====
        self.r = rospy.get_param("/Capybot/wheelRadius")
        self.d = rospy.get_param("/Capybot/dPoint")
        self.h = rospy.get_param("/Capybot/wheelDistance")
        self.m = rospy.get_param("/Capybot/mass")
        # I = 1  

        # ===== Rate =====
        self.rate = rospy.Rate(repsInSec)
        self.dt = 1.0/repsInSec
        self.dt_odom = 1.0/odomFreq
        
        # ===== Shutdown =====
        rospy.on_shutdown(self.stop)

        # ===== Class attributes =====

        # Aka. q (estado inicial)
        self.xP = np.array([[0.0, 0.0, 0.0]]).T

        self.K = np.array([[1, 0, 0], 
                            [0, 2, 0], 
                            [0, 0, 0]])

        # Aka z
            # In this case our sensors are exactly the dynamics
        # Sensor vector
            # x
            # y
            # z, but needed for multiplication
        self.sensorVect = np.array([[0.0, 0.0, 0.0]]).T

        # Aka H
            # Identity matrix for sensor input
            # Vector space from a virtual sensor
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
        
        # System dynamics
        self.A = np.array([[self.r/2*np.cos( self.xP[2,0] )-self.h*self.r/self.d*np.sin( self.xP[2,0] ),
                            self.r/2*np.cos( self.xP[2,0] )+self.h*self.r/self.d*np.sin( self.xP[2,0] ), 0],
                        [self.r/2*np.sin( self.xP[2,0] ) + self.h*self.r/self.d*np.cos( self.xP[2,0] ), 
                            self.r/2*np.sin( self.xP[2,0] ) - self.h*self.r/self.d*np.cos( self.xP[2,0] ), 0],
                        [self.r/self.d, 
                            -self.r/self.d, 0]])


    def runKalman(self):
        pass
        
        # Recieving updates from sensors 
        # It's done on the callbacks :D


    def get_pose(self, msg):
        # Update sensor readout
        self.sensorVect[0,0] = msg.x
        self.sensorVect[1,0] = msg.y
        self.sensorVect[2,0] = msg.theta

        # Update prediction 
        self.xP = self.xP + self.dt_odom*( 
            np.matmul(self.A, self.xP) +
            # Kalman filter stuff
            np.matmul(self.prediction_Cov_Mat,
                np.matmul(self.statesSensor.T,
                        np.matmul(np.linalg.inv(self.sensor_Cov_Mat),
                            np.matmul((self.sensorVect-self.statesSensor,
                                self.xP))))))   

        # Update prediction covariance matrix
        self.prediction_Cov_Mat = self.prediction_Cov_Mat + self.dt_odom*(
                np.matmul(self.A, self.prediction_Cov_Mat) 
                    + np.matmul(self.prediction_Cov_Mat, self.A.T) 
                    + self.model_Cov_Mat -
                        np.matmul(self.prediction_Cov_Mat,
                        np.matmul(self.statesSensor.T,
                            np.matmul(
                                np.linalg.inv(self.sensor_Cov_Mat),
                                np.matmul(self.statesSensor,
                                        self.prediction_Cov_Mat)))))

    def stop(self):
        rospy.loginfo("Ended Kalman")

if __name__ == "__main__":
    kOdom = KalmanOdometry(25)
    while not rospy.is_shutdown():
        kOdom.runKalman()
        kOdom.rate.sleep()