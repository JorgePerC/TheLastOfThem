# Importar librerias
import numpy as np

# Importar dependencias de mensajes
from custom_msgs.msg import GridMap, Cell
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class Node():

  def __init__(self, id, pos):

    #Inicializar atributos
    self.id = id
    self.pos = pos 

    #Inicializar camino al origen
    self.path = []

class Branch():

  def __init__(self, start):

    #Crear primer nodo
    self.head = Node(0, start)
    self.head.path.append(0) 

    #Inicializar arreglo de nodos
    self.nodes = []
    self.nodes.append(self.head)

  #Encontrar punto mas cercano de la rama
  def find_nearest(self, point):

    #Inicializar arreglos auxiliares
    index = -1
    d = 10000.0

    #Encontrar punto mas cerano
    for i in range(len(self.nodes)):

      #Calcular distancia
      d_p = np.linalg.norm(self.nodes[i].pos - point) 
     # print(str(d_p) + " " + str(d))

      #Verificar condicion de minimo
      if(d_p < d):
        index = i
        d = d_p

    return index
    

  def add_node(self, point, obs, sec, d):

    #Encontrar punto mas cercano a la rama
    index = self.find_nearest(point)

    #Extrapolar punto
    p_new = RRT.extrapolate(self.nodes[index].pos, point, d)

    #Analizar obstaculos en busca de colisiones
    add_point = True 
    for i in range(obs.shape[0]):

      #Detectar colision entre linea y obstaculo i
      coll = RRT.det_collision(self.nodes[index].pos, p_new, obs[i:i+1, :], sec)

      #Si se detecta colision
      if(coll):
        add_point = False
        break

    #Agregar punto a la rama
    if(add_point):

      #Crear nuevo nodo
      nodo = Node(len(self.nodes), p_new) 

      #Heredar camino al origen
      nodo.path = self.nodes[index].path[:]
      nodo.path.append(len(self.nodes))

      #Agregar nodo a rama
      self.nodes.append(nodo)

class RRT():

  #Constructor
  def __init__(self, d, x_range, y_range, sec, it):
  
    #Maximo desplazamiento
    self.d = d

    #Rango de busqueda
    self.x_range = x_range
    self.y_range = y_range 

    #Zona de seguridad
    self.sec = sec

    #Limite de iteraciones
    self.it = it

    #Inicializar ramas
    self.Br = None
    self.Bt = None

  #Distancia euclidiana
  @staticmethod
  def euc(p1, p2):

    d = np.sqrt(np.power(p1[0,0] - p2[0,0], 2) + np.power(p1[0,1] - p2[0, 1], 2))

    return d

  #Extrapolar puntos
  @staticmethod
  def extrapolate(p1, p2, d):

    #Calcular deltas
    dy = p2[0, 1] - p1[0, 1]
    dx = p2[0, 0] - p1[0, 0]

    #Calcular longitud de la linea entre p1 y p2
    nl = np.linalg.norm(p2 - p1)

    #Extrapolar punto
    p3 = np.zeros([1, 2])
    p3[0, 0] = d*dx/nl + p1[0, 0]
    p3[0, 1] = d*dy/nl + p2[0, 1]

    return p3

  #Generar punto aleatorio
  def generate_point(self):

    #Inicializar punto
    point = np.zeros([1, 2])
    point[0, 0] = (self.x_range[1] - self.x_range[0])*np.random.rand(1) + self.x_range[0]
    point[0, 1] = (self.y_range[1] - self.y_range[0])*np.random.rand(1) + self.y_range[0]

    return point

  #Generacion de trayectorias
  def gen_traj(self, Po, Tg, Ob):

    #Inicializar ramas
    self.Br = Branch(Po)
    self.Bt = Branch(Tg) 

    #Inicializar trayectoria
    Et = []

    #Contador auxiliar
    it = 0

    #Ciclo principal
    while(it < self.it):

      #Generar punto aleatorio
      random_point = self.generate_point()

      #Conectar punto a las ramas
      self.Br.add_node(random_point, Ob, self.sec, self.d)
      self.Bt.add_node(random_point, Ob, self.sec, self.d)

      #Intentar conectar ramas
      
      #Encontrar punto mas cercano de la rama del objetivo
      index_target = self.Bt.find_nearest(self.Br.nodes[-1].pos)

      #Buscar colisiones con obstaculos del ambiente
      coll_branch = False 
      for i in range(Ob.shape[0]):

        #Detectar colision
        coll = RRT.det_collision(self.Br.nodes[-1].pos, 
                                 self.Bt.nodes[index_target].pos,
                                 Ob[i:i+1, :],
                                 self.sec)
        
        if(coll):
          coll_branch = True
          break

      #Si se pudieron conectar las ramas
      if(not(coll_branch)):

        #Obtener camino de las ramas
        path_robot = self.Br.nodes[-1].path 
        path_target = self.Bt.nodes[index_target].path

        #Agregar rama del robot (inicio -> fin)
        for i in range(len(path_robot)):

          Et.append(self.Br.nodes[path_robot[i]].pos)

        #Agregar rama del objetivo (fin -> inicio)
        for i in range(len(path_target)):

          Et.append(self.Bt.nodes[path_target[len(path_target) - i - 1]].pos) 

        #Romper ciclo
        break

      #Aumentar contador
      it += 1

    return np.array(Et)

  #Detectar colisiones entre puntos
  @staticmethod
  def det_collision(p1, p2, obs, sec):

    #Calcular lineas
    l1 = p2 - p1
    l2 = obs - p1

    #Calcular magnitud de la linea entre p1 y p2
    nl1 = np.linalg.norm(l1)

    #Calcular proyeccion de obstaculo sobre linea 1
    alpha = np.dot(l2, np.transpose(l1))/nl1

    #Calcular factor de interpolacion
    f = alpha/nl1

    #Calcular punto de la linea mas cercano al obstaculo
    po = p1 + f*(p2-p1)

    #Calcular punto medio
    pm = 0.5*(p1 + p2)

    #Calcular distancia de punto mas cercano al obstaculo
    d = np.linalg.norm(obs - po)

    #Punto dentro de la linea
    if(np.linalg.norm(po - pm) <= 0.5*nl1):
    
      #Verificar si punto esta dentro de zona de seguridad
      if(d <= sec):
        return True
      else:
        return False
        
    #Punto fuera de la linea
    else:
      #Verificar que limites de la linea no tengan colision
      if(np.linalg.norm(p1 - obs) <= 0.5*sec or np.linalg.norm(p2 - obs) <= 0.5*sec):
        return True
      else:
        return False

  #Simplificar trayectoria  
  def simplify_trajectory(self, traj, obs):

    #Inicializar trayectoria simplificada
    traj_opt = []

    #Inicializar trayectoria con primer punto
    traj_opt.append(traj[0, :, :]) 

    #Analizar trayectoria
    for i in range(1, traj.shape[0]):

      #Inicializar condicion de adicion
      add_point = False 

      #Buscar colisiones con los obstaculos
      for j in range(obs.shape[0]):

        #Verificar si existe colision
        collision = RRT.det_collision(traj_opt[-1], traj[i], obs[j:j+1, :], self.sec)

        #Establecer condicion
        if(collision):
          add_point = True
          break 

      #Verificar condicion de adicion
      if(add_point):
        traj_opt.append(traj[i-1])
    
    #Agregar punto final de la trayectoria
    traj_opt.append(traj[-1])

    return np.array(traj_opt)
  
# Convertir mapa de ocupacion en arreglo de obstaculos
def map2obs(map_msg, thresh):
    
  # Inicializar arreglo de obstaculos
  obs = []

  # Iterar sobre celdas de mapa
  for i in range(len(map_msg.data)):

    # Verificar threshold de probabilidad
    if(map_msg.data[i] > thresh):
      
      # Obtener punto en m
      x = map_msg.info.resolution * (i % map_msg.info.width) + map_msg.info.origin.position.x
      y = map_msg.info.resolution * (i // map_msg.info.width) + map_msg.info.origin.position.y
      
      # Generar obstaculo
      ob = np.array([[x, y, 0.0]])

      # Agregar a lista de obstaculos
      obs.append(ob)

  return np.array(obs)

# Convertir arreglo de puntos en marcadores de rviz
def array2rviz(array, marker, scale, color, time):

  # Inicializar mensaje de RViz
  marker_rviz = Marker()
  marker_rviz.header.stamp = time
  marker_rviz.header.frame_id = "map"
  marker_rviz.type = marker

  # Definir escala
  marker_rviz.scale.x = scale[0]
  marker_rviz.scale.y = scale[1]
  marker_rviz.scale.z = scale[2]

  # Definir color
  marker_rviz.color.r = color[0]
  marker_rviz.color.g = color[1]
  marker_rviz.color.b = color[2]
  marker_rviz.color.a = color[3]

  # Iterar sobre arreglo de numpy (arreglo de Nx1x2)
  for i in range(array.shape[0]):
      
    # Inicializar particula de Rviz
    point = Point()
    point.x = array[i, 0, 0]
    point.y = array[i, 0, 1]
    point.z = 0.0

    # Agregar punto a lista de rviz
    marker_rviz.points.append(point)
  
  return marker_rviz

# Verificar si trayectoria es valida
def check_trajectory(path, obs, sec):

  # Iterar sobre trayectoria (Nx1x2)
  for i in range(path.shape[0] - 1):
    
    # Iterar sobre arreglo de obstaculos
    for j in range(obs.shape[0]):
      
      # Detectar colision entre trama i -> i + 1 con obstaculo j
      coll = RRT.det_collision(path[i,:,0:2], path[i+1,:,0:2], obs[j,:,0:2], sec)

      if (coll):
        return True
      
  return False