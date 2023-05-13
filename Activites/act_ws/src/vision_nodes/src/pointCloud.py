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
#rospy.init_node.utils import hello

def map2Dto3D (z, xVector, fx= 379.2713, fy = 379.271, cx = 319.348, cy = 238.448):
  # fx = distancia focal (depende del lente)
  # cx, cy = Centro del punto en el eje
  # Parameters were setup for ZED Camera
  K = np.array([
                [fx,    0.0,   cx],
                [0.0,   fy,   cy],
                [0.0,   0.0,  1.0]])

  # Calcular la inversa para pasar de imagen a mundo real
  k_inv = np.linalg.inv(K)

  return z*np.dot(k_inv, xVector)

def makePointCloud (imgBGR, depth, a):
  pass

def main():

  #Inicializar nodo
  rospy.init_node("py_custom")

  #Leer parametros 
  imgsPath = rospy.get_param("/imgs_path", "../media")

  depthMap = cv2.imread(imgsPath+"/pr18.png", cv2.IMREAD_UNCHANGED)

  print(imgsPath+"/im18.png")
  imgRGB = cv2.imread(imgsPath+"/im18.png")

  

  #Crear instancia de clase Publisher
  #Inicializar indice de publicacion (timer)
  rate = rospy.Rate(15)
  
  # cv2.imshow("Foto RGB", imgRGB)
  # cv2.waitKey(-1)

  #Init 3d Point cloud
  points3D = [ ]

  for y in range (imgRGB.shape[0]):
    for x in range (imgRGB.shape[1]):
      
      # Crear 2D vector to representar pixel
        # invertimos para seguir la regla de la mano derecha
      pixel_2d = np.array([[x], [y], [1]])
      # Leer la profundidad (z), para poder hacer el calculo
        # Como esta en mm, pasamos a m
          # Pq rviz trabaja en m
      z = depthMap[y, x]*0.001

      # Calcular punto 3D
      p3D = map2Dto3D(z, pixel_2d)

      # Generamos el pixel con color y profundidad
      
      r = imgRGB[y, x, 2] # Red 
      g = imgRGB[y, x, 1] # Green
      b = imgRGB[y, x, 0] # Blue
      a = 255             # opacidad
      
      rgb  = struct.unpack("I", struct.pack("BBBB", b, g, r, a))[0]
      pt = [  p3D[0,0], p3D[1,0], p3D[2,0], rgb]
      points3D.append(pt) 
      #print(p3D)
  # Crear nube de puntos
  h = Header()
  h.frame_id = "world"

  f = [PointField("x", 0, PointField.FLOAT32, 1),
       PointField("y", 4, PointField.FLOAT32, 1),
       PointField("z", 8, PointField.FLOAT32, 1),
       PointField("rgba", 12, PointField.UINT32, 1)]
  pCloudMsg = point_cloud2.create_cloud(h, f, points3D)

  pClouds_pub = rospy.Publisher("/nubedCulos", PointCloud2, queue_size=1)
  
  # Crear ciclo
  while not rospy.is_shutdown():

    pCloudMsg.header.stamp = rospy.Time.now()
    pClouds_pub.publish(pCloudMsg)

    rate.sleep()
    
if __name__ == '__main__':
  main()
