#!/bin/bash

# Launch container and enable visual odometry

# Source capy_ws availabe packages
source /home/capybot/Documents/TheLastOfThem/capy-bot_ws/devel/setup.bash

# Launch controlCapy
roslaunch diff_drivetrain controlCapy.launch


