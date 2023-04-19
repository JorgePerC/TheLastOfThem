cmake_minimum_required(VERSION 3.0.2)
project(act0)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  # For custom messages
  custom_msgs # Package name for imported msgs
              # There is no need to include dependencies from the original 
              # package, since they've already been compiled as binaries
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES act0
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



##########
## Add executables and/or libraries ##

add_executable(
  talker 
  src/talker.cpp
)

add_executable(
  listener 
  src/listener.cpp
)

add_executable(
  talkerCustom 
  src/talkerCustom.cpp
)

target_link_libraries(
  talker
  ${catkin_LIBRARIES}
)

target_link_libraries(
  listener
  ${catkin_LIBRARIES}
)

target_link_libraries(
  talkerCustom
  ${catkin_LIBRARIES}
)

# Python nodes:
catkin_install_python(PROGRAMS
  scripts/pyCustTalker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)