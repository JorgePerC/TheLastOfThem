#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

class Sync:

    def __init__(self, r = 6.5):

        rospy.init_node("sync")

        self.sub = rospy.Subscriber("/robot/kalmanPose", Odometry, self.get_poseKalman)
        self.pub = rospy.Publisher("/syncOdom", Odometry, queue_size=5)

        self.rate = rospy.Rate(r)

        self.sync_msg = Odometry()

    def get_poseKalman(self, msg):
        self.sync_msg = msg

    def main(self):

        while not rospy.is_shutdown():
            self.sync_msg.header.stamp = rospy.Time.now()
            self.sync_msg.header.frame_id = "laser"
            self.pub.publish(self.sync_msg)
            self.rate.sleep()

if __name__ == "__main__":
    syncOdom = Sync(6.9)
    syncOdom.main()
