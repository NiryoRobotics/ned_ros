SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Can be changed, choices are: amd64 or arm64
ARCHITECTURE=amd64
VERSION=v3.2.0

DOCKER_IMAGE_NAME=registry.gitlab.com/niryo/niryo-one-s/ned_ros_stack/${VERSION}
## Tag to push image on gitlab
DOCKER_TAG=${ARCHITECTURE}

HOME_FOLDER=home
ROS_SRC_FOLDER=src
DOCKER_OUTPUT_HOST_FOLDER_NAME=docker
FOLDER_DEPENDENCIES_NAME=docker_ros_folder_dependencies

# Create temporary folder containing only package.xml files (preserving structure) just for rosdep install part in docker, so that it can be cached and not be reinstalled each time
cd ${SCRIPT_DIR}/../ && rm -rf ${FOLDER_DEPENDENCIES_NAME} && find . -type f -name "package.xml" | cpio -pdm ${FOLDER_DEPENDENCIES_NAME} && cd -;

build_docker()
{
    cross_build=$1
    if ! $cross_build; then
        ## Without buildx == classic build, for classic usage on native platform
        docker build --rm -t ${DOCKER_IMAGE_NAME}:${DOCKER_TAG} -f ${SCRIPT_DIR}/Dockerfile ${SCRIPT_DIR}/..
    else
        export DOCKER_BUILDKIT=0
        export COMPOSE_DOCKER_CLI_BUILD=0

        # Make sure multiplatform builder is enabled
        docker buildx inspect --bootstrap &> /dev/null

        # Multi platform builder is not enabled
        if ! ls /proc/sys/fs/binfmt_misc/ | egrep --quiet qemu-*; then
            echo "Starting multi platform builder..."
            docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
        fi

        docker buildx build --platform linux/$ARCHITECTURE --load --progress plain -t ${DOCKER_IMAGE_NAME}:${DOCKER_TAG} -f ${SCRIPT_DIR}/Dockerfile ${SCRIPT_DIR}/..
        if [ $? -ne 0 ]; then
            echo "If docker buildx is not installed, please refer to notion: https://www.notion.so/niryo/Docker-cff99457487740279ab98bbbd467d534"
        fi
    fi
}

current_architecture=`uname -m`

if [[ $ARCHITECTURE == "amd64" && $current_architecture == 'x86_64' ]]; then
    cross_build=false;
elif [[ $ARCHITECTURE == "arm64" && $current_architecture == 'aarch64' ]]; then
    cross_build=false;
else
    cross_build=true;
fi
build_docker $cross_build;

# Delete temporary folder when docker image builded
rm -rf ${SCRIPT_DIR}/../${FOLDER_DEPENDENCIES_NAME};
