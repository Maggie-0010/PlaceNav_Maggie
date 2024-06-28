#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/noetic/setup.bash" 
source "/opt/turtlebot_ws/devel/setup.bash"

if [ ! -d "/opt/placenav/src/placenav_viz_msgs/devel" ]; then 
    cd /opt/placenav/src/placenav_viz_msgs
    catkin_make
fi 

source "/opt/placenav/src/placenav_viz_msgs/devel/setup.bash"
exec "$@"
