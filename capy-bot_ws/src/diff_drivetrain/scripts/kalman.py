#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import tf

class KalmanOdometry:
    def __init__(self, odomFreq, repsInSec = 25):
        rospy.init_node("KalmanOdometry")
        rospy.loginfo("Starting ROSNode as Kalman odometry.")
        
        # ===== Subscribers =====
        self.sub_pose = rospy.Subscriber("/robot/pose", Pose2D, self.get_poseEncoder)
        self.sub_poseVisual = rospy.Subscriber("/robot/pose", Odometry, self.get_poseVisual)

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
        self.dt = 0.0
        self.lasTime = rospy.Time.now()
        
        # ===== Shutdown =====
        rospy.on_shutdown(self.stop)

        # ===== Class attributes =====

        # Aka. q (estado inicial)
        self.xP = np.array([[0.0, 0.0, 0.0]]).T

        self.K = np.array([[1, 0, 0], 
                            [0, 2, 0], 
                            [0, 0, 0]])

        # Aka Q
            # Covariance from model noise
        self.model_Cov_Mat = np.array([ [1.0, 0.0, 0.0],
                                        [0.0, 1.0, 0.0],
                                        [0.0, 0.0, 1.0]]) 
        # Aka H
            # Identity-ish matrix for sensor input
            # Vector space from sensor readouts to statesX
        self.statesSensor = np.array([  [1.0, 0.0, 0.0], 
                                        [0.0, 1.0, 0.0],
                                        [0.0, 0.0, 1.0]]) 
        # Aka P
            # Covariance from state estimation
            # This is system wide 
        self.prediction_Cov_Mat = np.array([ [0.0, 0.0, 0.0], 
                                            [0.0, 0.0, 0.0],
                                            [0.0, 0.0, 0.0]])
        """
        # System dynamics
        self.A = np.array(
                [[self.r/2*np.cos(self.xP[2,0]) - self.h*self.r/self.d*np.sin(self.xP[2,0]),
                        self.r/2*np.cos(self.xP[2,0]) + self.h*self.r/self.d*np.sin(self.xP[2,0]), 0],
                [self.r/2*np.sin(self.xP[2,0]) + self.h*self.r/self.d*np.cos(self.xP[2,0]), 
                        self.r/2*np.sin(self.xP[2,0]) - self.h*self.r/self.d*np.cos(self.xP[2,0]), 0],
                [self.r/self.d, 
                        -self.r/self.d, 0]])
	"""


    def runKalman(self):
        # We just use this to publish the new stimation
        msg2send = Pose2D()

        msg2send.x = self.xP[0,0]
        msg2send.y = self.xP[1,0]
        msg2send.theta = self.xP[2,0]
        
        self.pub_poseK.publish(msg2send)

    def updateDt(self):
        # This method should be called whenever 
        # there's a new sensor reading
        now = rospy.Time.now()
        self.dt = now.secs - self.lasTime.secs
        self.lasTime = now

    def get_poseVisual(self, msg):

        visOdom_sensor = np.array([[0.0, 0.0, 0.0]]).T
        # Update
        visOdom_sensor[0,0] = msg.pose.pose.position.x
        visOdom_sensor[1,0] = msg.pose.pose.position.y
        visOdom_sensor[2,0] = msg.pose.pose.orientation.w

        # Update time since last int
        self.updateDt()

        # System dynamics
        self.A = np.array(
                [[self.r/2*np.cos(self.xP[2,0]) - self.h*self.r/self.d*np.sin(self.xP[2,0]),
                        self.r/2*np.cos(self.xP[2,0]) + self.h*self.r/self.d*np.sin(self.xP[2,0]), 0],
                [self.r/2*np.sin(self.xP[2,0]) + self.h*self.r/self.d*np.cos(self.xP[2,0]), 
                        self.r/2*np.sin(self.xP[2,0]) - self.h*self.r/self.d*np.cos(self.xP[2,0]), 0],
                [self.r/self.d, 
                        -self.r/self.d, 0]])

        # Aka R
            # Covariance from virtual sensor 
            # In this case, we actually get the updates
                # From the odometry message
                # But for now we ignore it
        visualOdom_Cov_Mat = np.array([[0.2, 0.0, 0.0], 
                                        [0.0, 0.2, 0.0],
                                        [0.0, 0.0, 0.4]]) 
        # Update prediction 
        self.xP = self.xP + self.dt*( 
            np.matmul(self.A,self.xP) +
            # Kalman filter stuf
            np.matmul(self.prediction_Cov_Mat,
                np.matmul(self.statesSensor.T,
                    np.matmul(
                        np.linalg.inv(visualOdom_Cov_Mat),
                        (visOdom_sensor-np.matmul(self.statesSensor, 
                                                    self.xP))))))

        # Update prediction covariance matrix
        self.prediction_Cov_Mat = self.prediction_Cov_Mat + self.dt*(
                np.matmul(self.A, self.prediction_Cov_Mat) + 
                np.matmul(self.prediction_Cov_Mat, self.A.T) +
                self.model_Cov_Mat -
                np.matmul(self.prediction_Cov_Mat,
                    np.matmul(self.statesSensor.T,
                        np.matmul(np.linalg.inv(visualOdom_Cov_Mat),
                            np.matmul(self.statesSensor, 
                                    self.prediction_Cov_Mat)))))
        
        

    def get_poseEncoder(self, msg):
        # Aka z
            # In this case our sensors are exactly the dynamics
        # Encoder sensor vector
            # x
            # y
            # z, but needed for multiplication
                # Update sensor readout
        snrVectEncoders = np.array([[0.0, 0.0, 0.0]]).T

        snrVectEncoders[0,0] = msg.x
        snrVectEncoders[1,0] = msg.y
        snrVectEncoders[2,0] = msg.theta
        
        self.updateDt()
        
        # System dynamics
        self.A = np.array(
                [[self.r/2*np.cos(self.xP[2,0]) - self.h*self.r/self.d*np.sin(self.xP[2,0]),
                        self.r/2*np.cos(self.xP[2,0]) + self.h*self.r/self.d*np.sin(self.xP[2,0]), 0],
                [self.r/2*np.sin(self.xP[2,0]) + self.h*self.r/self.d*np.cos(self.xP[2,0]), 
                        self.r/2*np.sin(self.xP[2,0]) - self.h*self.r/self.d*np.cos(self.xP[2,0]), 0],
                [self.r/self.d, 
                        -self.r/self.d, 0]])


        # Aka R
            # Covariance from sensor input
            # This one is only applied t
        encoder_Cov_Mat = np.array([[0.1, 0.0, 0.0], 
                                        [0.0, 0.1, 0.0],
                                        [0.0, 0.0, 0.8]]) 
        # Update prediction 
        self.xP = self.xP + self.dt*( 
            np.matmul(self.A,self.xP) +
            # Kalman filter stuf
            np.matmul(self.prediction_Cov_Mat,
                np.matmul(self.statesSensor.T,
                    np.matmul(
                        np.linalg.inv(encoder_Cov_Mat),
                        (snrVectEncoders-np.matmul(self.statesSensor, 
                                                    self.xP))))))

        # Update prediction covariance matrix
        self.prediction_Cov_Mat = self.prediction_Cov_Mat + self.dt*(
                np.matmul(self.A, self.prediction_Cov_Mat) + 
                np.matmul(self.prediction_Cov_Mat, self.A.T) +
                self.model_Cov_Mat -
                np.matmul(self.prediction_Cov_Mat,
                    np.matmul(self.statesSensor.T,
                        np.matmul(np.linalg.inv(encoder_Cov_Mat),
                            np.matmul(self.statesSensor, 
                            self.prediction_Cov_Mat)))))
    def stop(self):
        rospy.loginfo("Ended Kalman")

if __name__ == "__main__":
    kOdom = KalmanOdometry(25)
    while not rospy.is_shutdown():
        kOdom.runKalman()
        kOdom.rate.sleep()
