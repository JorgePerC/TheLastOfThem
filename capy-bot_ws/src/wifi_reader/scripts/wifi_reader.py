#!/usr/bin/env python

import subprocess

import rospy
from std_msgs.msg import Float32

def get_wifi():
    # Example 2: Get a list of current tasks
    result = subprocess.run(["iwconfig"], shell=True)

    signal_strength = float(result)

    return signal_strength

if __name__ == "__main__":
    rospy.init_node("wifi_antenna")
    rospy.loginfo("Starting wifi_antenna.")

    node_pub = rospy.Publisher("/robot/antena", Float32, queue_size=1)
    
    node_rate = rospy.Rate(2)

    while not rospy.is_shutdown():

        r = Float32()

        r.data = get_wifi()
        node_pub.publish(r)

        rospy.spin()
        node_rate.sleep()


