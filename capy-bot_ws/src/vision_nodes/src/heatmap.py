#!/usr/bin/env python

#Importar rospy
import rospy 

#Importar utilidades
import numpy as np

from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D, PoseStamped
from custom_msgs.msg import Path
from vision_nodes.trajectory_algs import array2rviz_array

class HeatMap():
    def __init__(self):
        # Inicializar nodo
        rospy.init_node("heat_map")
        
        # Wait got
        rospy.wait_for_message("/map", OccupancyGrid, 10)
        rospy.wait_for_message("/trajectory_gen/path_msg", Path, 10)
        # Inicializar subscribers
        rospy.Subscriber("/map", OccupancyGrid, self.mapCallback)
        rospy.Subscriber("/robot/antena", Float32, self.signalCallback)
        rospy.Subscriber("/trajectory_gen/path_msg", Path, self.pathCallback)
        rospy.Subscriber("/slam_out_pose", PoseStamped, self.robot_poseCallback)
        
        
        # Inicializar publisher
        self.heat_map_pub = rospy.Publisher("/heat_map", MarkerArray, queue_size=1)
        self.point_pub = rospy.Publisher("/robot/human/objective", Pose2D, queue_size=1)
        
        # Mapa
        self.map = OccupancyGrid()
        self.dim_map = 0
        
        # Heat map
        self.dim_grid = 5
        self.grid = []
        self.signal = 0

        # Trajectory
        self.path = None
        
        # Robot
        self.robot_pose = PoseStamped()
        
        # Generate points:
        self.idxGrid = 0
        self.get_map()
        self.hasRecievedRRT = False
        # Send initial point
        self.sendPoint()

        # Rate
        self.rate = rospy.Rate(20)

    def sendPoint(self):
        msg = Pose2D()
        msg.x = self.grid[self.idxGrid][0][0]
        msg.y = self.grid[self.idxGrid][0][1]
        msg.theta = 0
        # If point is 00, ignore
        #if self.grid[self.idxGrid][0][0] == 0 and self.grid[self.idxGrid][0][0] == 0:
            #return
        print(msg.x, msg.y)
        print(self.idxGrid)
        self.point_pub.publish(msg)
        self.hasRecievedRRT = False

    def robotIsClose(self):
        distance = np.sqrt(pow((self.robot_pose.pose.position.x - self.grid[self.idxGrid][0][0]),2) +  pow((self.robot_pose.pose.position.y - self.grid[self.idxGrid][0][1]),2))
        return distance < 0.1
        
    def main(self):
        while not rospy.is_shutdown():
            self.get_map()
            # Wait if hasn't arrived
            if (self.idxGrid==0 and np.sqrt(pow((self.robot_pose.pose.position.x - 0.0),2) +  pow((self.robot_pose.pose.position.y - 0.0),2)) < 0.1):
                self.sendPoint()
            #if (self.idxGrid == 0):
                #self.sendPoint()
            if (self.hasRecievedRRT and len(self.path.path) != 0):
                # If robot is not close
                if not self.robotIsClose():
                    continue
                # Robot has arrived
                else:
                    # Save reading 
                    self.heat_rviz.markers[self.idxGrid].color.r = (-2.0*self.signal/255.0)
                    print("Reading: ", self.signal)
                    # Also send new point
                    self.idxGrid = self.idxGrid + 1
                    self.sendPoint()
            # If there is no path to our idx, increment it
            if (self.hasRecievedRRT and len(self.path.path) == 0):
                self.idxGrid = self.idxGrid + 1
                self.sendPoint()

            # Always display heatmap   
            self.heat_rviz = array2rviz_array(np.array(self.grid), 1, 
                                   [self.map.info.resolution*self.dim_map/self.dim_grid, self.map.info.resolution*self.dim_map/self.dim_grid, 0.1],
                                     [1.0, 150.0/255.0, 60.0/255.0, 0.7], rospy.Time.now())
            
            self.heat_map_pub.publish(self.heat_rviz)

            # Do while-ish
                # End if we have sent all the points
                # Or master is shutdown
            if self.idxGrid == len(self.grid)**2 :
                break
            self.rate.sleep()

    def get_map(self):
        self.grid = []
        for i in range(self.dim_grid,0,-1):
            for j in range(self.dim_grid,0,-1):
                i_grid = int((0.5+i-1)*(self.dim_map/self.dim_grid))
                j_grid = int((0.5+j-1)*(self.dim_map/self.dim_grid))
                x = self.map.info.resolution * i_grid + self.map.info.origin.position.x
                y = self.map.info.resolution * j_grid + self.map.info.origin.position.y
                self.grid.append([[x,y]])
        
    
    def mapCallback(self, map_msg):
        self.map = map_msg
        self.dim_map = map_msg.info.width
    
    def signalCallback(self, signal_msg):
        self.signal = signal_msg

    def pathCallback(self, path_msg):
        self.path = path_msg
        self.hasRecievedRRT = True
    
    def robot_poseCallback(self, pose_msg):
        self.robot_pose = pose_msg


if __name__ == '__main__':
    hm = HeatMap()
    hm.main()
