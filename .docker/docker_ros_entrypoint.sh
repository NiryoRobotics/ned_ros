#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

CATKIN_WS_PATH=/home/niryo/catkin_ws

mkdir -p ${CATKIN_WS_PATH};
cd ${CATKIN_WS_PATH};
exec "$@"

