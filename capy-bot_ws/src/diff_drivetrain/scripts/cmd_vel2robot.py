#!/usr/bin/env python
import rospy
import math

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class Cmd_velControl:
    def __init__(self, scale):
        rospy.init_node("commandCapywheels")
        rospy.loginfo("Starting Cmd_velControl as name_node.")

        self.scale = scale
        # ===== Subscribers =====
        self.sub_cmd = rospy.Subscriber("/turtle1/cmd_vel", Twist, self.get_cmd)
        
        # ===== Publishers =====
        self.pub_wl = rospy.Publisher("/robot/set_wl", Float32, queue_size = 5)
        self.pub_wr = rospy.Publisher("/robot/set_wr", Float32, queue_size = 5)
    
    def get_cmd(self, msg):
        
        x = msg.linear.x
        w = msg.angular.z

        wl = self.scale*(2*x - w)/2
        wr = self.scale*(2*x + w)/2

        self.pub_wl.publish(wl)
        self.pub_wr.publish(wr)


if __name__ == "__main__":
    name_node = Cmd_velControl(2)
    rospy.spin()