#!/usr/bin/env python

import os

import rospy
from std_msgs.msg import Float32

def get_wifi():
    # Example 2: Get a list of current tasks
    result = os.popen("iwconfig").read()
    a = result.split("Signal level=") 
    num = a[-1].split("dBm")
    signal_strength = int(num[0])

    return signal_strength

if __name__ == "__main__":
    rospy.init_node("wifi_antenna")
    rospy.loginfo("Starting wifi_antenna.")

    node_pub = rospy.Publisher("/robot/antena", Float32, queue_size=1)
    
    node_rate = rospy.Rate(2                                               )

    while not rospy.is_shutdown():

        r = Float32()

        r.data = get_wifi()
        node_pub.publish(r)

        node_rate.sleep()