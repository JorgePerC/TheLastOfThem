#!/usr/bin/env python

# Importar rospy
import rospy

# Librerias basicas
import numpy as np
import cv2
import matplotlib.pyplot as plt

# Librerias custom
from vision_nodes.utils import OccGrid
from vision_nodes.conversions import quaternion_to_euler

# Importar mensajes
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from custom_msgs.msg import Cell, GridMap

# Importar utilidad de mensajes sincronizados
import message_filters

# Subscriber
def LaserCallback(msg_odom, msg_laser):
    global edwin, grid_pub
    print("Message")

    # Extraer datos de odometria (traslacion)
    x = msg_odom.pose.pose.position.x
    y = msg_odom.pose.pose.position.y
    z = msg_odom.pose.pose.position.z

    # Extraer datos de odometria (orientacion)
    qx = msg_odom.pose.pose.orientation.x
    qy = msg_odom.pose.pose.orientation.y
    qz = msg_odom.pose.pose.orientation.z
    qw = msg_odom.pose.pose.orientation.w

    # Convertir quaternion a angulos de Euler
    theta = quaternion_to_euler([qx, qy, qz, qw])

    # Construir estado de posicion
    state = [x, y, theta[0]]

    #print("Odometria:", state)

    # Obtener informacion del laser
    laser_ranges = msg_laser.ranges
    laser_angles = np.linspace(msg_laser.angle_min, msg_laser.angle_max, len(laser_ranges))

    # Construir estado del laser
    state_laser = np.vstack([laser_angles, laser_ranges])

    print("Laser:", state_laser)
    print("==================================")

    # Actualizar mapa de ocupacion
    edwin.update_laser(state, state_laser)

    # Obtener mapa de ocupacion
    occ_map = edwin.get_map()

    # Obtener mensaje de Rviz
    occ_rviz = edwin.get_rviz(occ_map, rospy.Time.now())

    # Publicar mensaje
    marker_pub.publish(occ_rviz)

    # Convertir mapa de ocupacion
    grid_msg = edwin.map_msg(occ_map, rospy.Time.now())

    # Publicar gradilla (mensaje)
    grid_pub.publish(grid_msg)

def main():

    global edwin, marker_pub
    global display, rviz
    global grid_pub
    
    # Inicializar nodo
    rospy.init_node("occupacy_node")
    rospy.loginfo("Laser map initialized")

    # Leer parametros de archivo de configuracion
    ## 1- Topicos de entrada
    input_odom = rospy.get_param("/occupacy_grid/topics/odom", "/robot/odom")
    input_laser = rospy.get_param("/occupacy_grid/topics/laser", "/robot/laser")

    ## 2- Parametros de la gradilla
    x_grid = rospy.get_param("/occupacy_grid/grid/x", 10)
    y_grid = rospy.get_param("/occupacy_grid/grid/y", 10)
    d_grid = rospy.get_param("/occupacy_grid/grid/dim", 1.0)

    ## 3- Parametros del algoritmo
    p_thresh = rospy.get_param("/occupacy_grid/params/p_thresh", 0.8)
    p_occ = rospy.get_param("/occupacy_grid/params/p_occ", 0.66)
    p_free = rospy.get_param("/occupacy_grid/params/p_free", 0.34)
    z_max = rospy.get_param("/occupacy_grid/params/z_max", 10.0)

    ## 4- Configuracion de interface
    display = rospy.get_param("/occupacy_grid/interface/display", True)
    rviz = rospy.get_param("/occupacy_grid/interface/rviz", True)

    # Crear publishers
    marker_pub  = rospy.Publisher("/occupacy_grid/rviz_grid", MarkerArray, queue_size=1)
    grid_pub = rospy.Publisher("/occupacy_grid/grid_map", GridMap, queue_size=1)

    # Crear instancia de mapa de ocupacion
    edwin = OccGrid(x_g = x_grid, y_g=y_grid, d_g=d_grid)

    # Crear subscribers
    odom_sub = message_filters.Subscriber(input_odom, Odometry)
    laser_sub = message_filters.Subscriber(input_laser, LaserScan)

    # Crear sincronizador de mensajes
    ts = message_filters.ApproximateTimeSynchronizer([odom_sub, laser_sub], queue_size=2, slop=0.5, allow_headerless=True)

    # Registrar callback
    ts.registerCallback(LaserCallback)

    # Llamar a callback
    rospy.spin()

if __name__ == '__main__':
    main()
