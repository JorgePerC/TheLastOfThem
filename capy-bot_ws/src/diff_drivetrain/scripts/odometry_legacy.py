#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
import numpy as np

class Odometry:

    def __init__(self, tmsInSec = 25):
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
        self.pub_pose = rospy.Publisher("/robot/pose", Pose2D, queue_size=10)

        # ===== Params =====
        self.r = rospy.get_param("/Capybot/wheelRadius")
        self.d = rospy.get_param("/Capybot/dPoint")
        self.h = rospy.get_param("/Capybot/wheelDistance")

        self.pose = Pose2D()
        self.pose.x = 0
        self.pose.y = 0
        self.pose.theta = 0

        # ===== Rate =====
        self.rate = rospy.Rate(tmsInSec)
        self.dt = 1.0/tmsInSec

        # ===== Shutdown =====
        rospy.on_shutdown(self.stop)
    
    def wr_callback(self, velocity):
        self.wr = velocity.data

    def wl_callback(self, velocity):
        self.wl = velocity.data

    def stop(self):
        rospy.loginfo("Ended odometry")

    def main(self):
        while not rospy.is_shutdown():
            
            distance = self.r * (self.wr + self.wl) * 0.5 * self.dt
            self.pose.theta += self.r * (self.wr - self.wl) / self.l * self.dt

            if self.pose.theta < -np.pi:
                self.pose.theta += 2*np.pi
            elif self.pose.theta > np.pi:
                self.pose.theta -= 2*np.pi

            xd = distance*np.cos(self.pose.theta)
            yd = distance*np.sin(self.pose.theta)

            self.pose.x += xd
            self.pose.y += yd

            self.pub.publish(self.pose)

            self.rate.sleep()


if __name__ == '__main__':
    try:
        odom = Odometry()
        odom.main()

    except rospy.ROSInterruptException:
        pass