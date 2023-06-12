#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose2D, PoseStamped
from custom_msgs.msg import Path
from std_msgs.msg import Bool

class PointsArray:
    def __init__(self, repsInSec = 25):
        rospy.init_node("pathPlannig")
        rospy.loginfo("Start Path Planning, expecting array of points and publishing when finish")

        #Subscribers
        self.sub_points = rospy.Subscriber("/trajectory_gen/path_msg", Path, self.get_points)
        self.sub_bool = rospy.Subscriber("/robot/bool", Bool, self.get_bool)
        self.sub_traj_flag = rospy.Subscriber("/trajectory_gen/new_traj", Bool, self.get_traj_flag)
        self.sub_pose = rospy.Subscriber("/slam_out_pose", PoseStamped, self.poseCallback)
        
        #Publisher
        self.pub_point = rospy.Publisher("/robot/objective", Pose2D, queue_size=5)

        #Flags
        self.pub_goal = rospy.Publisher("/robot/path_ended", Bool, queue_size=1)
        self.pub_flag = rospy.Publisher("/robot/bool", Bool, queue_size=1)
        self.pub_traj_flag = rospy.Publisher("/trajectory_gen/new_traj", Bool, queue_size=1)

        #Rate
        self.rate = rospy.Rate(repsInSec)

        # Variables
        self.Points = []
        self.flag = True
        self.traj_flag = True
        self.path_flag = False
        self.poseRobot = PoseStamped()
    
    def pathPlanning(self):
        

        if self.flag and self.Points:
            self.pose = Pose2D()
            self.pose.x = self.Points[0].x
            self.pose.y = self.Points[0].y
            self.pose.theta = 0
            self.pub_point.publish(self.pose)
            print(self.Points.pop(0))
            self.flag = False
            self.pub_flag.publish(self.flag) 
            #self.pub_goal.publish(False)    
        if not self.Points:
            self.pose = Pose2D()
            self.pose.x = self.poseRobot.pose.position.x
            self.pose.y = self.poseRobot.pose.position.y
            self.pose.theta = 0
            self.pub_point.publish(self.pose)


        #elif self.flag and not self.Points:
            #self.pub_goal.publish(True)


    def get_points(self, msg):
        if self.traj_flag:
            self.Points = msg.path
            self.traj_flag = False
            self.pub_traj_flag.publish(self.traj_flag)


    def get_bool(self, msg):
        self.flag = msg.data
    
    def get_traj_flag(self, msg):
        self.traj_flag = msg.data
    
    def poseCallback(self, pose_msg):
        self.poseRobot = pose_msg

if __name__ == '__main__':
    Plan = PointsArray(repsInSec=25)
    while not rospy.is_shutdown():
        Plan.pathPlanning()
        Plan.rate.sleep()