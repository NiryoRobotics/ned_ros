#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

if [ -f ${HOME}/catkin_ws/devel/setup.bash ]; then
    source "${HOME}/catkin_ws/devel/setup.bash"
fi

mkdir -p ${HOME}/catkin_ws;
cd ${HOME}/catkin_ws;
exec "$@"