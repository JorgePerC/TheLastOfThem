cmake_minimum_required(VERSION 3.0.2)
project(oswaldo)

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
  custom_msgs
)

## Agreagar librerias
find_package(OpenCV REQUIRED)

## Dependencias de catkin
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES oswaldo
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Cabeceras
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Agregar nodos de C++
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})

add_executable(camera_pub src/camera_pub.cpp)
target_link_libraries(camera_pub ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(camera_subscriber src/camera_subscriber.cpp)
target_link_libraries(camera_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(marker_pub src/marker_pub.cpp)
target_link_libraries(marker_pub ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(talker_custom src/talker_custom.cpp)
target_link_libraries(talker_custom ${catkin_LIBRARIES} ${OpenCV_LIBS})

# Python nodes
catkin_install_python(PROGRAMS
  src/talker_custom.py
  DESTINATION $(CATKIN_PACKAGE_BIN_DESTINATION))