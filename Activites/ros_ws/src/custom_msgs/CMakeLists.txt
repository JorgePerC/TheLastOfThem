cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Agregar dependencias de catkin
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  image_transport
  cv_bridge
  visualization_msgs
  message_generation
)

# Mensajes a compilar
add_message_files(
  FILES
  pixel.msg
  point.msg
)

# Dependencias de los mensajes
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)





## Dependencias de catkin
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES oswaldo
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
