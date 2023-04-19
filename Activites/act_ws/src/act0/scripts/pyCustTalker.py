#!/usr/bin/env python
import rospy

import numpy as np
from custom_msgs.msg import point

if __name__ == "__main__":
    rospy.init_node("pyCustomTalker")
    rospy.loginfo("Starting pyCustomTalker.")

    message_pub = rospy.Publisher("customTopic", point, queue_size=10)
    
    rate = rospy.Rate(15)

    msg = point()
    
    while not rospy.is_shutdown():
        msg.pimg.v.data = np.random.randint(-20,10)
        msg.pimg.u.data = np.random.randint(-15,10)

        msg.pose.position.x = np.random.randint(-20,10)
        msg.pose.position.y = np.random.randint(-20,10)
        msg.pose.position.z = np.random.randint(-20,10)

        message_pub.publish(msg)
        print("Running")
        rate.sleep()
