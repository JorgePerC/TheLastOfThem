#!/usr/bin/env python

# Importar rospy
import rospy 

# Importar utilidades
import numpy as np

# Importar dependencias de mensajes
from custom_msgs.msg import GridMap, Cell, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

# Importar libreria de generacion de trayectorias
from vision_nodes.trajectory_algs import RRT
from vision_nodes.trajectory_algs import map2obs, array2rviz, check_trajectory

# Callback de mapa de ocupacion
def MapCallback(map_msg):
  
  global obs_pub, traj_pub, path_pub
  global traj_gen, recalc_trajectory, Et
  global p_start, p_end, sec
  
  # Convertir obstaculos a arreglo de numpy
  obs = map2obs(map_msg, 0.8)

  if (recalc_trajectory):

    # Parametros del generador
    start = np.array([p_start])
    target = np.array([p_end])

    # Calcular trayectoria
    Et = traj_gen.gen_traj(start, target, np.array(obs)[:,0,0:2])
    if Et.shape[0] > 0:
      Et = traj_gen.simplify_trajectory(Et, np.array(obs)[:,0,0:2])
      recalc_trajectory = False
  
  else:

    # Verificar si trayectoria calculada no tiene colisiones
    coll = check_trajectory(Et, obs, sec)

    if (coll):
      recalc_trajectory = True

  # Convertir obstaculos y trayectoria en mensaje de rviz
  obs_rviz = array2rviz(obs, 7, [0.5, 0.5, 0.5], [0.0, 1.0, 0.0, 0.7], rospy.Time.now())
  path_rviz = array2rviz(Et, 4, [0.1, 0.0, 0.0], [1.0, 1.0, 1.0, 0.7], rospy.Time.now())
  
  # Publicar trayectoria
  path_msg = Path()
  path_msg.header.stamp = rospy.Time.now()
  path_msg.path = path_rviz.points

  # Publicar mensaje
  obs_pub.publish(obs_rviz)

  # Publicar trayectoria en rviz
  traj_pub.publish(path_rviz)
  path_pub.publish(path_msg)

  #print("Obstaculos encontrados: " + str(len(obs)))

def main():
  
  global traj_pub, obs_pub, path_pub
  global traj_gen, recalc_trajectory, Et
  global p_start, p_end, sec

  #I nicializar nodo
  rospy.init_node("trajectory_gen")

  # Leer parametros de archivo de configuracion
  map_topic = rospy.get_param("/trajectory_rrt/topics/input_map", "/occupacy_grid/grid_map")
  
  # Parametros del RRT
  d = rospy.get_param("/trajectory_rrt/rrt/d", 0.5)
  x_range = rospy.get_param("/trajectory_rrt/rrt/x_range", [-2, 2])
  y_range = rospy.get_param("/trajectory_rrt/rrt/y_range", [0, 6])
  sec = rospy.get_param("/trajectory_rrt/rrt/sec", 0.5)
  it = rospy.get_param("/trajectory_rrt/rrt/it", 100)

  # Requerimientos de trayectoria
  p_start = rospy.get_param("/trajectory_rrt/path/start", [0.0, 0.0])
  p_end = rospy.get_param("/trajectory_rrt/path/end", [2.0, 0.0])

  # Inicializar subscriber
  rospy.Subscriber(map_topic, GridMap, MapCallback)

  # Crear instancia de clase Publisher
  traj_pub = rospy.Publisher("/trajectory_gen/path", Marker, queue_size=1)
  obs_pub = rospy.Publisher("/trajectory_gen/obs", Marker, queue_size=1)
  path_pub = rospy.Publisher("/trajectory_gen/path_msg", Path, queue_size=1)

  # Crear instancia de algoritmo
  traj_gen = RRT(d, x_range, y_range, sec, it)

  # Incializar bandera
  recalc_trajectory = True

  # Inicializar indice de publicacion (timer)
  rate = rospy.Rate(15)

  rospy.spin()
    
if __name__ == '__main__':
  main()
