#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf
import math
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

def quaternion_to_euler(q):
    #Get quaternion
    (x, y, z, w) = (q[0], q[1], q[2], q[3])

    #Calculate angles
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    
    return [yaw, pitch, roll] 

class KalmanOdometry:
    def __init__(self, repsInSec = 40):
        rospy.init_node("KalmanOdometry")
        rospy.loginfo("Starting ROSNode as Kalman odometry.")
        
        # ===== Subscribers =====
            # Needed for space stimation
        self.sub_wr = rospy.Subscriber("/robot/wr", Float32, self.get_wl)
        self.sub_wl = rospy.Subscriber("/robot/wl", Float32, self.get_wr)
        self.sub_pose = rospy.Subscriber("/robot/noisyOdom", Odometry, self.get_poseEncoder)
        self.sub_poseVisual = rospy.Subscriber("/rtabmap/odom", Odometry, self.get_poseVisual)
        #self.sub_imu = rospy.Subscriber("/imu/data", Imu, self.get_imu)
        
        # ===== Publishers =====
        self.pub_poseK = rospy.Publisher("/robot/kalmanPose", Odometry, queue_size=5)
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

        self.U = np.array([[0.0, 0.0]]).T

        self.K = np.array([ [1, 0, 0], 
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
        self.prediction_Cov_Mat = np.array([[0.0, 0.0, 0.0], 
                                            [0.0, 0.0, 0.0],
                                            [0.0, 0.0, 0.0]])
        rospy.sleep(1)
        np.seterr(divide='ignore', over = 'ignore', under = 'ignore', invalid= 'ignore')
    
    def get_wr(self, msg):
        self.U[0, 0] = msg.data 
        
    def get_wl(self, msg):
        self.U[1, 0] = msg.data

    def runKalman(self):
        # We just use this to publish the new stimation
        cy = np.cos(self.xP[2,0] * 0.5)
        sy = np.sin(self.xP[2,0] * 0.5)

        msg2send = Odometry()
        
        msg2send.header.frame_id = "world"
        msg2send.header.stamp = rospy.Time.now()

        msg2send.pose.pose.position.x = self.xP[0,0]
        msg2send.pose.pose.position.y = self.xP[1,0]

        q = euler2quaternion([self.xP[2, 0], 0.0, 0.0])

        msg2send.pose.pose.orientation.x = q[0]
        msg2send.pose.pose.orientation.y = q[1]
        msg2send.pose.pose.orientation.z = q[2]
        msg2send.pose.pose.orientation.w = q[3] 


        self.pub_poseK.publish(msg2send)

    def updateDt(self):
        # This method should be called whenever 
        # there's a new sensor reading
        now = rospy.Time.now()
        self.dt =  (now.nsecs - self.lasTime.nsecs)/1000000000.0
        self.lasTime = now

    def updateA(self):
        self.A = np.array(
                [[self.r/2 - 0, self.r/2 + 0],
                [0 + self.h*self.r/self.d, 0 - self.h*self.r/self.d],
                [self.r/self.d, 
                        -self.r/self.d]])

    def get_imu(self, msg):
        #Update time
        self.updateDt()
        # System dynamics
        self.updateA()

        imu_sensor = np.array([[0.0, 0.0, 0.0]]).T
        
        int_accX = msg.linear_acceleration.z*self.dt
        int_accY = msg.linear_acceleration.y*self.dt

        # Update
        imu_sensor[0,0] = int_accX
        imu_sensor[1,0] = int_accY
        imu_sensor[2,0] = msg.angular_velocity.x

        
        gyro_Cov_Mat = np.array([[100.0, 0.0, 0.0], 
                                [0.0, 100.0, 0.0],
                                [0.0, 0.0, 10.0]]) 
        # Update prediction 
        self.xP = self.xP + self.dt*( 
            np.matmul(self.A, self.xP) +
            # Kalman filter stuf
            np.matmul(self.prediction_Cov_Mat,
                np.matmul(self.statesSensor.T,
                    np.matmul(
                        np.linalg.inv(gyro_Cov_Mat),
                        (imu_sensor - np.matmul(self.statesSensor, 
                                                    self.xP))))))

        # Update prediction covariance matrix
        self.prediction_Cov_Mat = self.prediction_Cov_Mat + self.dt*(
                np.matmul(self.A, self.prediction_Cov_Mat) + 
                np.matmul(self.prediction_Cov_Mat, self.A.T) +
                self.model_Cov_Mat -
                np.matmul(self.prediction_Cov_Mat,
                    np.matmul(self.statesSensor.T,
                        np.matmul(np.linalg.inv(gyro_Cov_Mat),
                            np.matmul(self.statesSensor, 
                                    self.prediction_Cov_Mat)))))

    def get_poseVisual(self, msg):
        # Aka z
            # In this case our sensors are exactly the dynamics
        visOdom_sensor = np.array([[0.0, 0.0, 0.0]]).T
        # Update
        visOdom_sensor[0,0] = msg.pose.pose.position.x
        visOdom_sensor[1,0] = msg.pose.pose.position.y
        l = [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w]
        thetas = quaternion_to_euler(l)
        visOdom_sensor[2,0] = thetas[0]

        # To avoid crashing the estimation
        if np.isnan(visOdom_sensor[0,0]):
            print("Skipped visual Odom bc it has nulls")
            return

        # Update time
        self.updateDt()
        # In case negative, skip integration
        if  self.dt <= 0:
            print("Problem with dt")
            return

        # System dynamics
        self.updateA()

        # Aka R
            # Covariance from sensor input
            # This one is only applied t
        visualOdom_Cov_Mat = np.array([ [0.5, 0.0, 0.0], 
                                        [0.0, 0.5, 0.0],
                                        [0.0, 0.0, 0.3]]) 
        
        # Update prediction 
        self.xP = self.xP + self.dt*( 
            np.matmul(self.A, self.U) +
            np.matmul(self.prediction_Cov_Mat,
                np.matmul(np.linalg.inv(visualOdom_Cov_Mat),
                    (visOdom_sensor - self.xP))))
        
        # Update prediction covariance matrix
        self.prediction_Cov_Mat = self.prediction_Cov_Mat + self.dt*(
            self.model_Cov_Mat -
            np.matmul(self.prediction_Cov_Mat,
                np.matmul(np.linalg.inv(visualOdom_Cov_Mat),
                        self.prediction_Cov_Mat)))

    def get_poseEncoder(self, msg):
        # Aka z
            # In this case our sensors are exactly the dynamics
        # Encoder sensor vector
            # x
            # y
            # z, but needed for multiplication
                # Update sensor readout
        l = [msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w]

        thetas = quaternion_to_euler(l)

        snrVectEncoders = np.array([[msg.pose.pose.position.x, 
                                    msg.pose.pose.position.y, 
                                    thetas[0]]]).T
        # Update time
        self.updateDt()
        # In case negative, skip integration
        if  self.dt <= 0:
            return
        # System dynamics
        self.updateA()

        # Aka R
            # Covariance from sensor input
            # This one is only applied t
        encoder_Cov_Mat = np.array([[0.5, 0.0, 0.0], 
                                    [0.0, 0.5, 0.0],
                                    [0.0, 0.0, 0.8]]) 
        
        # Update prediction 
        self.xP = self.xP + self.dt*( 
            np.matmul(self.A, self.U) +
            # Kalman filter stuf
            np.matmul(self.prediction_Cov_Mat,
                #np.matmul(self.statesSensor.T,
                    np.matmul(
                        np.linalg.inv(encoder_Cov_Mat),
                        (snrVectEncoders - self.xP))))
        
        # Update prediction covariance matrix
        self.prediction_Cov_Mat = self.prediction_Cov_Mat + self.dt*(
            # np.matmul(self.A, self.prediction_Cov_Mat) + 
            # np.matmul(self.prediction_Cov_Mat, self.A.T) +
            self.model_Cov_Mat -
            np.matmul(self.prediction_Cov_Mat,
                #np.matmul(self.statesSensor.T,
                    np.matmul(np.linalg.inv(encoder_Cov_Mat),
                            self.prediction_Cov_Mat)))#)
        
    def stop(self):
        rospy.loginfo("Ended Kalman")

if __name__ == "__main__":
    kOdom = KalmanOdometry()
    while not rospy.is_shutdown():
        kOdom.runKalman()
        kOdom.rate.sleep()
