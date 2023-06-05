#Importar rospy
import rospy 

#Importar utilidades
import numpy as np
import cv2
import struct

#Librerias custom
from vision_nodes.utils import hello

# Importar mensajes
from std_msgs.msg import Header
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField

def main():

  #Inicializar nodo
  rospy.init_node("py_custom")

  #Leer parametros de archivo de configuracion
  node_path = rospy.get_param("/p_cloud/node_path", "~/media")
  print("Ruta del paquete:" + node_path)
  #Crear instancia de clase Publisher
  
  # Leer imagen
  img_rgb = cv2.imread(node_path + "/media/im18.png")
  depth_map = cv2.imread(node_path + "/media/pr18.png", cv2.IMREAD_UNCHANGED)

  print(depth_map)

  # Crear publisher

  pc_pub = rospy.Publisher("/point_cloud_py", PointCloud2, queue_size=1)

  # Mostrar en pantalla
  #cv2.imshow("img_rgb", img_rgb)
  #cv2.imshow("depth_map", depth_map)
  #cv2.waitKey(-1)

  # Definir parametros de calibracion
  K = np.array([[379.2713, 0.0, 319.348],
               [0.0, 379.271, 238.448],
               [0.0, 0.0, 1]])
  
  K_inv = np.linalg.inv(K)

  # Matriz de rotacion imagen -> Rviz
  theta = -np.pi / 2
  R_c_rviz = np.array([[1.0, 0.0, 0.0], [0.0, np.cos(theta), -np.sin(theta)], [0.0, np.sin(theta), np.cos(theta)]])

  # Inicializar lista de puntos en 3D
  points_3d = []

  # Calcular nube de puntos
  for i in range(img_rgb.shape[0]):
    for j in range(img_rgb.shape[1]):
      # Crear vector de punto en 2D
      p_2d = np.array([[j], [i], [1]])
      
      # Calcular punto en 3D
      z = 0.001* depth_map[i, j] # Profundidad en milimetros
      p_3d = z*np.dot(K_inv, p_2d)
      p_3d = R_c_rviz@p_3d
      
      # Extraer color de la imagen
      b = img_rgb[i, j, 0]
      g = img_rgb[i, j, 1]
      r = img_rgb[i, j, 2]
      a = 255

      # Construir punto
      rgb = struct.unpack("I", struct.pack('BBBB', b, g, r, a))[0]
      pt = [p_3d[0,0], p_3d[1,0], p_3d[2,0], rgb]
      points_3d.append(pt)

  # Crear cabecera del mensaje
  header = Header()
  header.frame_id = "world"

  # Crear campos
  fields = [PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgba', 12, PointField.UINT32, 1)]

  # Crear nube de puntos
  pc_msg = point_cloud2.create_cloud(header, fields, points_3d)

  #Inicializar indice de publicacion (timer)
  rate = rospy.Rate(15)

  #Crear ciclo
  while not rospy.is_shutdown():

    # Estampar mensaje
    pc_msg.header.stamp = rospy.Time.now()
    pc_pub.publish(pc_msg)
    
    #Retardo
    rate.sleep()
    
if __name__ == '__main__':
  main()
