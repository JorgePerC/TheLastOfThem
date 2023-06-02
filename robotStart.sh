#!/bin/bash

# Move into capybot workspace
cd /home/capybot/Documents/TheLastOfThem/capy-bot_ws/

# Source capy_ws availabe packages
source devel/setup.bash

# Launch realsense camera
roslaunch realsense2_camera rs_camera.launch 

# Launch controlCapy
#roslaunch diff_drivetrain kapy.launch


