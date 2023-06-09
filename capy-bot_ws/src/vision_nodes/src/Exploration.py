#!/usr/bin/env python
import rospy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError

class NewHorizon:
    def __init__(self):
        rospy.init_node("new_horizon")
        rospy.loginfo("Starting node as new_horizon.")
        rospy.Subscriber("/map", OccupancyGrid, self.createMasks)
        rospy.Subscriber("/path_ended", Bool, self.get_Flag)

        self.mSize = 128 #rospy.get_param("/hector_mapping/map_size", 128)
        self.probObs = 75 #rospy.get_param("/visionNodes/threshold", 75)

        self.k = np.ones((3,3),np.uint8)
        self.shouldSend = False

        self.points_pub = rospy.Publisher("/robot/human/reference", Pose2D, queue_size=10)
    
    def get_Flag(self, msg):
        self.shouldSend = msg.data

    def createMasks(self, msg):
        # If not set flag, break
        if not(self.shouldSend):
            return

        # Get map data
        arr =  np.asarray(msg.data)
        # Shape it to an image
        map = np.reshape(arr, (self.mSize, self.mSize)).astype(np.uint8)

        # Get masks
        obsMask = cv.inRange(map, self.probObs, 150)
        # Manage -1s
        map[map == -1] = 251 
        ret, knowMask = cv.threshold(map, self.probObs, 255,cv.THRESH_BINARY_INV)
        ret, unkwonMask = cv.threshold(map, 250, 255, cv.THRESH_BINARY)
        
        # Dilate borders
        # obsMask = cv.dilate(obsMask, self.k, iterations = 1)
        # unkwonMask = cv.dilate(unkwonMask, self.k, iterations = 1)
        knowMask = cv.dilate(knowMask, self.k, iterations = 1)
        # Mask
        noGoZone = cv.bitwise_or(obsMask, unkwonMask)
        prev = cv.bitwise_and(knowMask, noGoZone)
        mask = cv.bitwise_xor(obsMask, prev)

        mask = cv.dilate(mask, self.k, iterations = 2)
        # Find countours
        contours, hierarchy = cv.findContours(mask, method=cv.CHAIN_APPROX_SIMPLE, mode=cv.RETR_EXTERNAL)[-2:]

        result = cv.cvtColor(mask, cv.COLOR_GRAY2BGR)
        
        points = []
        for cont in contours:
            M = cv.moments(cont)
            for i in M:
                # Avoid division by 0
                if M[i] == 0:
                    M[i] += 1e-5
            # Filter by area
            if M['m00'] < 250:
                continue
            # Get center point
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # Draw middle point
            cv.circle(result, (cX, cY), 5, (255, 0, 0), -1)

            points.append([cX, cY])
        
        # There is no point to sent
        if len(points) == 0:
            return
        
        send = Pose2D
        send.x = points[0][0]
        send.y = points[0][1]
        send.theta = 0
        self.points_pub.publish(send)
        cv.imshow("try", result)
        cv.waitKey(0)

if __name__ == "__main__":
    name_node = NewHorizon()
    rospy.spin()