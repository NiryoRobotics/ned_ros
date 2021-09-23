# Can be changed, choices are: amd64 or arm64
ARCHITECTURE=amd64
VERSION=v3.2.0

DOCKER_IMAGE_NAME=registry.gitlab.com/niryo/niryo-one-s/ned_ros_stack/${VERSION}

## Tag to push image on gitlab
DOCKER_TAG=${ARCHITECTURE}

PATH_TO_SOURCE_FOLDER=`pwd`
DOCKER_OUTPUT_HOST_FOLDER_NAME=docker

HOME_FOLDER=home/niryo

ROS_SRC_FOLDER=src

# Multi platform builder is not enabled
if ! ls /proc/sys/fs/binfmt_misc/ | egrep --quiet qemu-*; then
    echo "Starting multi platform builder..."
    docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
fi

# If source located under current . folder
if [ -d ${PATH_TO_SOURCE_FOLDER}/niryo_robot_bringup ]; then
    docker run --rm --platform linux/${ARCHITECTURE} -it \
    -v ${PATH_TO_SOURCE_FOLDER}:/${HOME_FOLDER}/catkin_ws/src \
    -v ${PATH_TO_SOURCE_FOLDER}/../${DOCKER_OUTPUT_HOST_FOLDER_NAME}/${ARCHITECTURE}/build:/${HOME_FOLDER}/catkin_ws/build \
    -v ${PATH_TO_SOURCE_FOLDER}/../${DOCKER_OUTPUT_HOST_FOLDER_NAME}/${ARCHITECTURE}/devel:/${HOME_FOLDER}/catkin_ws/devel \
    -p 9090:9090 \
    -p 11311:11311 \
    ${DOCKER_IMAGE_NAME}:${DOCKER_TAG} $@
# If source located under ./src folder
elif [ -d ${PATH_TO_SOURCE_FOLDER}/src ]; then
    docker run --rm --platform linux/${ARCHITECTURE} -it \
    -v ${PATH_TO_SOURCE_FOLDER}/src:/${HOME_FOLDER}/catkin_ws/src \
    -v ${PATH_TO_SOURCE_FOLDER}/${DOCKER_OUTPUT_HOST_FOLDER_NAME}/${ARCHITECTURE}/build:/${HOME_FOLDER}/catkin_ws/build \
    -v ${PATH_TO_SOURCE_FOLDER}/${DOCKER_OUTPUT_HOST_FOLDER_NAME}/${ARCHITECTURE}/devel:/${HOME_FOLDER}/catkin_ws/devel \
    -p 9090:9090 \
    -p 11311:11311 \
    ${DOCKER_IMAGE_NAME}:${DOCKER_TAG} $@
else
    echo "Incorrect PATH_TO_SOURCE_FOLDER setted, cannot find ROS sources: ${PATH_TO_SOURCE_FOLDER}"
fi
