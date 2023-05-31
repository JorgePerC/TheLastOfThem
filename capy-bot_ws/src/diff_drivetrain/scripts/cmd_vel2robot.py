#!/usr/bin/env python
import rospy
import numpy as np

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class Cmd_velControl:
    def __init__(self, scale):
        rospy.init_node("commandCapywheels")
        rospy.loginfo("Starting Cmd_velControl as name_node.")

        # ===== Subscribers =====
        self.sub_cmd = rospy.Subscriber("/turtle1/cmd_vel", Twist, self.get_cmd)
        
        # ===== Publishers =====
        self.pub_wl = rospy.Publisher("/robot/set_wl", Float32, queue_size = 5)
        self.pub_wr = rospy.Publisher("/robot/set_wr", Float32, queue_size = 5)
        
        # ===== Params =====
        self.d = rospy.get_param("/Capybot/dPoint")
        self.r = rospy.get_param("/Capybot/wheelRadius")
        
        # ===== Class attributes =====
        self.cmd = np.array([[0.0, 0.0]]).T
        self.scale = scale
    
    def get_cmd(self, msg):
        
        self.cmd[0,0] = msg.linear.x
        self.cmd[1,0]  = msg.angular.z
        # Send control
        self.set_smooth_cmd

    def set_smooth_cmd(self):
        # Older method:

        # # wl = self.scale*(2*x - w)/2
        # # wr = self.scale*(2*x + w)/2

        # New method
        A = np.array([[self.r/2, self.r/2],
                      [self.r/self.d, -self.r/self.d]])

        u = np.dot( np.linalg.inv(A), self.cmd)
        self.pub_wr.publish(u[0,0])
        self.pub_wl.publish(u[1,0])
        

if __name__ == "__main__":
    name_node = Cmd_velControl(2)
    rospy.spin()