#!/usr/bin/env python

# Importar rospy
import rospy 

# Importar utilidades
import numpy as np

# Importar dependencias de mensajes
from custom_msgs.msg import GridMap, Cell, Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose2D, PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

# Importar libreria de generacion de trayectorias
from vision_nodes.trajectory_algs import RRT
from vision_nodes.trajectory_algs import map2obs, array2rviz, check_trajectory

class Traj_gen:
  def __init__(self):
    #Inicializar nodo
    rospy.init_node("trajectory_gen")

    # Leer parametros de archivo de configuracion
    map_topic = rospy.get_param("/trajectory_rrt/topics/input_map", "/occupacy_grid/grid_map")
    
    # Parametros del RRT
    d = rospy.get_param("/trajectory_rrt/rrt/d", 0.5)
    x_range = rospy.get_param("/trajectory_rrt/rrt/x_range", [-2, 2])
    y_range = rospy.get_param("/trajectory_rrt/rrt/y_range", [0, 6])
    self.sec = rospy.get_param("/trajectory_rrt/rrt/sec", 0.5)
    it = rospy.get_param("/trajectory_rrt/rrt/it", 100)

    # Requerimientos de trayectoria
    self.p_start = rospy.get_param("/trajectory_rrt/path/start", [0.0, 0.0])
    self.p_end = rospy.get_param("/trajectory_rrt/path/end", [0.0, 0.0])

    # Inicializar subscriber
    rospy.Subscriber(map_topic, OccupancyGrid, self.MapCallback)
    rospy.Subscriber("/robot/human/objective", Pose2D, self.goalCallback)
    rospy.Subscriber("/slam_out_pose", PoseStamped, self.originCallback)

    # Crear instancia de clase Publisher
    self.traj_pub = rospy.Publisher("/trajectory_gen/path", Marker, queue_size=1)
    self.obs_pub = rospy.Publisher("/trajectory_gen/obs", Marker, queue_size=1)
    self.path_pub = rospy.Publisher("/trajectory_gen/path_msg", Path, queue_size=1)
    self.flag_pub = rospy.Publisher("/trajectory_gen/new_traj", Bool, queue_size=1)

    # Crear instancia de algoritmo
    self.traj_gen = RRT(d, x_range, y_range, self.sec, it)

    # Incializar bandera
    self.recalc_trajectory = True
    self.new_traj_flag = False
    self.flag_pub.publish(self.new_traj_flag)

    # Empty path
    self.Et = []

    rospy.spin()

  # Callback de mapa de ocupacion
  def MapCallback(self, map_msg):
    
    # Convertir obstaculos a arreglo de numpy
    obs = map2obs(map_msg, 75)

    if (self.recalc_trajectory or self.new_traj_flag):

      # Parametros del generador
      start = np.array([self.p_start])
      target = np.array([self.p_end])

      # Calcular trayectoria
      self.Et = self.traj_gen.gen_traj(start, target, np.array(obs)[:,0,0:2])
      if self.Et.shape[0] > 0:
        self.Et = self.traj_gen.simplify_trajectory(self.Et, np.array(obs)[:,0,0:2])
        self.recalc_trajectory = False
        self.new_traj_flag = False
        self.flag_pub.publish(not self.new_traj_flag)
    
    else:

      # Verificar si trayectoria calculada no tiene colisiones
      coll = check_trajectory(self.Et, obs, self.sec)

      if (coll):
        self.recalc_trajectory = True

    # Convertir obstaculos y trayectoria en mensaje de rviz
    obs_rviz = array2rviz(obs, 7, [0.1, 0.1, 0.1], [1.0, 0.0, 0.0, 0.7], rospy.Time.now())
    path_rviz = array2rviz(self.Et, 4, [0.1, 0.0, 0.0], [1.0, 1.0, 1.0, 0.7], rospy.Time.now())
    
    # Publicar trayectoria
    path_msg = Path()
    path_msg.header.stamp = rospy.Time.now()
    path_msg.path = path_rviz.points

    # Publicar mensaje
    self.obs_pub.publish(obs_rviz)

    # Publicar trayectoria en rviz
    self.traj_pub.publish(path_rviz)
    self.path_pub.publish(path_msg)

    #print("Obstaculos encontrados: " + str(len(obs)))

  def originCallback(self, origin_msg):
    self.p_start = [origin_msg.pose.position.x, origin_msg.pose.position.y]

  def goalCallback(self, goal_msg):
    self.p_end = [goal_msg.x, goal_msg.y]
    self.new_traj_flag = True
    
if __name__ == '__main__':
  gen = Traj_gen()