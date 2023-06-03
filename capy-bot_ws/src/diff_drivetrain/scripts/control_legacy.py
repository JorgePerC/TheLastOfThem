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
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #rospy.Subscriber("/position", Twist, self.pos_callback)
        #rospy.Subscriber("/step", Twist, self.step_callback)
        #ospy.Subscriber("/go", Bool, self.go_callback)


        self.sub_pose = rospy.Subscriber("/robot/pose", Pose2D, self.get_poseRobot)
        self.sub_poseD = rospy.Subscriber("/robot/objective", Pose2D, self.get_poseDeseada)

        # # ===== Rate =====
        self.rate = rospy.Rate(repsInSec)
        self.dt = 1.0/repsInSec

        rospy.on_shutdown(self.stop)

        # ===== Params =====
        self.r = rospy.get_param("/Capybot/wheelRadius")
        self.d = rospy.get_param("/Capybot/dPoint")
        self.h = rospy.get_param("/Capybot/wheelDistance")

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

    def pos_callback(self, pos):
        self.x = pos.linear.x
        self.y = pos.linear.y
        self.theta = pos.angular.z

    def step_callback(self, goal):
        self.xd = goal.x
        self.yd = goal.y

    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.pub.publish(self.vel)

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

                if v > 0.5:
                    v = 0.5
                if omega > pi/8:
                    omega = pi/8
                elif omega < -pi/8:
                    omega = -pi/8

                #v = v*(1-tanh(abs(alpha)*2))

                self.vel.linear.x = v
                self.vel.angular.z = omega

                self.pub.publish(self.vel)

            else:
                self.stop()

            rospy.on_shutdown(self.stop)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = Control(0.7, 0.2, 40, 0.01)
        node.main()

    except rospy.ROSInterruptException:
        pass    