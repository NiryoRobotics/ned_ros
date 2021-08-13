#!/bin/bash

if [ $# -eq 0 ]; then
    echo "You must provide one argument: ROBOT_IP";
    exit 1;
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

DOCKER_OUTPUT_HOST_FOLDER_NAME=docker

ROBOT_IP="$1"
TMP_PASSWORD_FILE=.tmp_password

# If sshpass not installed
if [ ! -x "$(command -v sshpass)" ]; then
    echo "Installing sshpass..."
    sudo apt install -y --no-install-recommends sshpass;
fi

export SSHPASS="robotics"
sshpass -e rsync -avz $SCRIPT_DIR/../../$DOCKER_OUTPUT_HOST_FOLDER_NAME/* niryo@$ROBOT_IP:/home/niryo/catkin_ws/test;
# rsync src folder
sshpass -e rsync -avz $SCRIPT_DIR/../* niryo@$ROBOT_IP:/home/niryo/catkin_ws/test/src/