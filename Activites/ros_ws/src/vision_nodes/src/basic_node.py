#Importar rospy
import rospy 

#Importar utilidades
import numpy as np

#Librerias custom
from vision_nodes.utils import hello

#Importar mensajes
from custom_msgs.msg import point

def main():

  #Inicializar nodo
  rospy.init_node("py_custom")

  #Leer parametros de archivo de configuracion
  ox = rospy.get_param("/talker_py/orbit/x", 1.0)
  oy = rospy.get_param("/talker_py/orbit/y", 1.0)
  oz = rospy.get_param("/talker_py/orbit/z", 1.0)

  #Crear instancia de clase Publisher
  emisor = rospy.Publisher("/talker/msg", point, queue_size=1)

  #Inicializar indice de publicacion (timer)
  rate = rospy.Rate(15)

  #Tiempo simulado
  t = 0.0

  #Inicializar mensaje
  msg = point()
  
  #Llamar a funcion custom
  hello()

  #Crear ciclo
  while not rospy.is_shutdown():
  
    #Llenar atributos
    msg.pimg.u.data = 50
    msg.pimg.v.data = 100
    msg.pose.position.x = ox*np.sin(t)
    msg.pose.position.y = -oy*np.cos(t)
    msg.pose.position.z = oz*np.sin(t)
    msg.header.stamp = rospy.get_rostime()
    
    #Publicar
    emisor.publish(msg)

    #Tiempo simulado
    t = t + 1.0/15.0
    
    #Retardo
    rate.sleep()
    
if __name__ == '__main__':
  main()
