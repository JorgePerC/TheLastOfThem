#!/usr/bin/env python
#Importar rospy
import rospy 

#Importar utilidades
import numpy as np
import cv2

import struct
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField

#librerias custom
from vision_nodes.utils import VisPointCloud
from sensor_msgs.msg import PointCloud2, PointField

def callback_RGB():
    pass

def callback_depth():
    pass
def main():

    #Inicializar nodo
    rospy.init_node("py_custom")

    #Leer parametros para la matriz
    camChar = rospy.get_param("/camChars", [1, 1, 0, 0])

    scale = rospy.get_param("/scale", 0)
    k = rospy.get_param("/k", 0)

    # Los topicos nos lo dan parámetros 
    topicRGB = rospy.get_param("/p_cloud_node/topics/rgb", "camera_raw", callback_RGB)
    topicDepth = rospy.get_param("/p_cloud_node/topics/depth", "depth", callback_depth)
    topicResult = rospy.get_param("/p_cloud_node/topics/out", "pcloud")

    # Generar publishers y subscribers
    rgbSub = rospy.Subscriber(topicRGB, )
    depthSub = rospy.Subscriber(topicDepth, PointField, )
    pClouds_pub = rospy.Publisher(topicResult, PointCloud2, queue_size=1)

    #Inicializar indice de publicacion (timer)
    rate = rospy.Rate(15)

    # Instancia de vision:

    steroCamish = VisPointCloud(i for i in camChar)
    
    # Crear ciclo
    while not rospy.is_shutdown():

        pCloudMsg.header.stamp = rospy.Time.now()
        pClouds_pub.publish(pCloudMsg)

        rate.sleep()
        
if __name__ == '__main__':
    main()
