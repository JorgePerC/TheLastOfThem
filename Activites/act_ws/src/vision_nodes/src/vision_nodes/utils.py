#!/usr/bin/env python
#Importar rospy
import rospy 

#Additional libs
import numpy as np

import cv2

import struct
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField

#Functions
def hello():
    print("Hello!")
  
class VisPointCloud():

    def __init__(self,  fx= 379.2713, fy = 379.271, cx = 319.348, cy = 238.448):
        self.K = np.array([
                    [fx,    0.0,   cx],
                    [0.0,   fy,   cy],
                    [0.0,   0.0,  1.0]])

        self.k_inv = np.linalg.inv(self.K)
    
        # Rotacion camera-mundo
        theta = -3.1416/2
        self.R = np.array([
                    [1.0,    0.0,   0.0],
                    [0.0,  np.cos(theta), -np.sin(theta)],
                    [0.0,  np.sin(theta),  np.cos(theta)]])


    """
    Método para calcular la nube de puntos
    """
    def map2Dto3D(self, z, xVector):
        return z*np.dot(self.k_inv, xVector)
    
    """
    Method to calculate point cloud
    Input:
    img: RGB img
    depth: depth map
    k: density
    d_scale: depth scale (m)
    Output:
    pCloud: point cloud
    """
    def makePointCloud(self, img, depth, k = 2, d_scale = 0.001):
        """
        A trick to improve the performance of the algorithm is to 
        skip data points (in this case pixels)
        """

        pCloud = []
        for y in range (0, img.shape[0], k):
            for x in range (0, img.shape[1], k):
            
            # Crear 2D vector to representar pixel
                # invertimos para seguir la regla de la mano derecha
                pixel_2d = np.array([[x], [y], [1]])

                if depth[y, x] < 0:
                    return
                else:

                    z = depth[y, x]*d_scale

                    # Calcular punto 3D
                    p3D = self.map2Dto3D(z, pixel_2d)
                    
                    # Tranformada para rotar en RViz
                    p3D = np.dot(self.R, p3D)

                    # Generamos el pixel con color y profundidad
                    r = img[y, x, 2] # Red 
                    g = img[y, x, 1] # Green
                    b = img[y, x, 0] # Blue
                    a = 255             # opacidad
                    
                    rgb  = struct.unpack("I", struct.pack("BBBB", b, g, r, a))[0]
                    pt = [  p3D[0,0], p3D[1,0], p3D[2,0], rgb]

                    pCloud.append(pt) 
        return pCloud
    
    def pCloud2pCloudMsg(self, pCloud, frame):

        # Crear nube de puntos
        h = Header()
        h.frame_id = frame

        f = [PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgba", 12, PointField.UINT32, 1)]
        pCloudMsg = point_cloud2.create_cloud(h, f, pCloud)

        return pCloudMsg