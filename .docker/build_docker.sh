#!/bin/bash

# Default values
DOCKER_REGISTRY_URL=gitlab01.niryotech.com:5050/robot/ned/ned-ros-stack
DOCKER_IMAGE_NAME=ned_ros_stack
DOCKER_TAG=latest
PUSH_IMAGE=false

# Parse short and long options
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -r|--registry) DOCKER_REGISTRY_URL="$2"; shift ;;
        -i|--image) DOCKER_IMAGE_NAME="$2"; shift ;;
        -t|--tag) DOCKER_TAG="$2"; shift ;;
        -p|--push) PUSH_IMAGE=true ;;
        -h|--help)
            echo "Usage: $0 [-r|--registry DOCKER_REGISTRY_URL] [-i|--image DOCKER_IMAGE_NAME] [-t|--tag DOCKER_TAG] [-p|--push]"
            exit 0
            ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
DOCKER_IMAGE_PATH="${DOCKER_REGISTRY_URL}/${DOCKER_IMAGE_NAME}:${DOCKER_TAG}"

# Build the Docker image
docker build --rm -t "${DOCKER_IMAGE_PATH}" -f "${SCRIPT_DIR}/Dockerfile" "${SCRIPT_DIR}/.."

# Push the Docker image if the --push | -p argument is given
if [ "$PUSH_IMAGE" = true ]; then
    docker push "${DOCKER_IMAGE_PATH}"
fi
