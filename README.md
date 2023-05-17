# TheLastOfThem
Our final project in university. 



## What this is about:
* Build a differential rover
* It's ment to generate rich maps about WiFi coverage

### Authors:
@ Luis Vaca
@ Josue Mojica
@ Jorge Pérez

Owen nos la pela


## Some dependencies: 

To pull the submodules included use:

    git submodule update --init --recursive

Also, you may need to install from APT:

* ros-melodic-mrpt-slam
* ros-melodic-depthimage-to-laserscan
* ros-melodic-realsense2-camera (Follow additional instructions for Jetson Devices)
* ros-melodic-rosserial

In case something didn't work:

    * ros-melodic-mrpt-sensors
    * ros-melodic-mrpt-msgs
    * ros-melodic-mrpt-msgs-bridge
    * libmrpt-dev mrpt-apps
    * fkie_multimaster_msgsC
    * mrpt-guiConfig
    * ros-melodic-multimaster-fkie 

## BEFORE BUILDING

Add mrpt_graphslam_2d to the catkin exclude 

    catkin config --skiplist mrpt_graphslam_2d

Change the branch on realsense-ros

    cd src/realsense-ros/
    git checkout ros1-legacy

