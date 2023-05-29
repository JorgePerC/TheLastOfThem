# TheLastOfThem
Our final project in university. 



## What this is about:
* Build a differential rover
* It's ment to generate rich maps about WiFi coverage
* Part of this proyect is located in the [CapyWheels repo](https://github.com/JorgePerC/CapyWheels.git)


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

You may need to downgrade your OpenCv version depending on the image installed. Check [this](https://answers.ros.org/question/347754/jetson-nano-comes-with-opencv-411-do-i-need-to-downgrade-to-32-for-melodic/) for help
