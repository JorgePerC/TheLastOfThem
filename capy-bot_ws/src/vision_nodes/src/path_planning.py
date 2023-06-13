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
            # Get path points
        self.sub_points = rospy.Subscriber("/trajectory_gen/path_msg", Path, self.get_points)
            # Get control feedback
        self.sub_control = rospy.Subscriber("/robot/isAtTarget", Bool, self.get_controlFlag)
            # Get robot pose
        self.sub_pose =  rospy.Subscriber("/slam_out_pose", PoseStamped, self.get_robotPose)
            # To know when there's a new path
        self.sub_hasNewTraj = rospy.Subscriber("/trajectory_gen/new_traj", Bool, self.get_newTrajFlag)
            # Also, send that it has been updated
        self.pub_newTraj = rospy.Publisher("/trajectory_gen/new_traj", Bool, queue_size=1)
        #Publisher
            # For control node
        self.pub_objective = rospy.Publisher("/robot/objective", Pose2D, queue_size=5)
        #Flags
        self.f_ctrlEnd = True
        self.f_hasNewTraj = True
        

        # Variables
        self.Points = []
        
        self.path_flag = False
        # Robot
        self.poseRobot = Pose2D()
        self.poseDeseasa = Pose2D()
        #Rate
        self.rate = rospy.Rate(repsInSec)

    def robotIsClose(self):
        distance = np.sqrt(pow((self.poseRobot.x - self.poseDeseasa.x),2) +  pow((self.poseRobot.y - self.poseDeseasa.y),2))
        print("_____________________")
        print("Distance: ", distance)
        return distance < 0.1
    
    def pathPlanning(self):
        # If control hasn't reached the point
        # Do nothing 
        if not(self.f_ctrlEnd):
            return
        
        # First check if there are no points in 
        # new trayectory or in the desired one.
        if len(self.Points) == 0:
            self.poseDeseasa.x = self.poseRobot.x
            self.poseDeseasa.y = self.poseRobot.y
            self.poseDeseasa.theta = 0
            
            print("No more points, robot at: ", self.poseRobot)
            # Prepare itself, to recieve new paths
            self.pub_newTraj.publish(True)
            # This should be updated by the same node
            #hasNewTraj = True
        # else send next point
        else:
            # Rewrite pose deseada and resent it
            sendMe = self.Points.pop(0)
            # Como llega points
            self.poseDeseasa.x = sendMe.x
            self.poseDeseasa.y = sendMe.y
            self.poseDeseasa.theta = 0

            self.pub_objective.publish(self.poseDeseasa)

            print(sendMe)
    
    # This callback executes when 
    def get_points(self, msg):
        if self.f_hasNewTraj:
            self.Points = msg.path
            self.f_hasNewTraj = False
            self.pub_newTraj.publish(False)

    def get_newTrajFlag(self, msg):
        self.f_hasNewTraj = msg.data
    #
    def get_controlFlag(self, msg):
        # Each time the control ends, it will publish so in here
        self.f_ctrlEnd = msg.data
    
    
    def get_robotPose(self, pose_msg):
        self.poseRobot.x = pose_msg.pose.position.x
        self.poseRobot.y = pose_msg.pose.position.y
        # No theta

if __name__ == '__main__':
    Plan = PointsArray(repsInSec = 5)
    while not rospy.is_shutdown():
        Plan.pathPlanning()
        Plan.rate.sleep()