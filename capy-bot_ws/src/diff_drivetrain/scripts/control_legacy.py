#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
#from std_msgs.msg import Bool
from math import *
from geometry_msgs.msg import Pose2D


class Control:
    def __init__(self, kt, kr, repsInSec, threshold):
        rospy.init_node("poseControl")
        self.pub_wl = rospy.Publisher("/robot/set_wl", Float32, queue_size=5)
        self.pub_wr = rospy.Publisher("/robot/set_wr", Float32, queue_size=5)
        #rospy.Subscriber("/position", Twist, self.pos_callback)
        #rospy.Subscriber("/step", Twist, self.step_callback)
        #ospy.Subscriber("/go", Bool, self.go_callback)
        self.scale = 2

        self.sub_pose = rospy.Subscriber("/robot/pose", Pose2D, self.get_poseRobot)
        self.sub_poseD = rospy.Subscriber("/robot/objective", Pose2D, self.get_poseDeseada)

        # # ===== Rate =====
        self.rate = rospy.Rate(repsInSec)
        self.dt = 1.0/repsInSec


        # ===== Params =====
        #self.r = rospy.get_param("/Capybot/wheelRadius")
        #self.d = rospy.get_param("/Capybot/dPoint")
        #self.h = rospy.get_param("/Capybot/wheelDistance")

        self.threshold = threshold

        self.x = 0
        self.y = 0
        self.theta = 0

        self.kt = kt
        self.kr = kr

        self.xd     = 0
        self.yd     = 0

        self.vel = Twist()
        self.vel.linear.x = 0
        self.vel.linear.y = 0
        self.vel.linear.z = 0
        self.vel.angular.x = 0
        self.vel.angular.y = 0
        self.vel.angular.z = 0
	
        rospy.on_shutdown(self.stop)


    def get_poseRobot(self, pos):
        self.x = pos.x
        self.y = pos.y
        self.theta = pos.theta

    def get_poseDeseada(self, goal):
        self.xd = goal.x
        self.yd = goal.y

    def stop(self):
        self.pub_wl.publish(0)
        self.pub_wr.publish(0)

    def main(self):
        while not rospy.is_shutdown():

            if sqrt((self.xd-self.x)**2 + (self.yd-self.y)**2) >= self.threshold:
                
                xe = self.x-self.xd
                ye = self.y-self.yd

                thetad = atan2(self.yd-self.y,self.xd-self.x)
                thetae = self.theta - thetad
                
                if abs(thetae) > pi:
                    thetae = (thetae/abs(thetae))*(abs(thetae)-2*pi)
                    
                v = self.kt*sqrt(xe**2+ye**2)
                omega = -self.kr*thetae

                if v > 2.0:
                    v = 2.0
                if omega > pi/4:
                    omega = pi/4
                elif omega < -pi/4:
                    omega = -pi/4

                #v = v*(1-tanh(abs(alpha)*2))

                self.vel.linear.x = v
                self.vel.angular.z = omega

                wl = self.scale*(2*v - omega)/2
                wr = self.scale*(2*v + omega)/2

                self.pub_wl.publish(wl)
                self.pub_wr.publish(wr)

            else:
                self.stop()

            rospy.on_shutdown(self.stop)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Control(kt =10.0, kr =6, repsInSec=40, threshold=0.05)
        node.main()

    except rospy.ROSInterruptException:
        pass    
