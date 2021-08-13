ARG ROS_DISTRO=melodic
FROM ros:${ROS_DISTRO}-perception AS BUILD_DEPENDENCIES

ENV HOME=/home/niryo
WORKDIR ${HOME}

ENV FOLDER_DEPENDENCIES_NAME=docker_ros_folder_dependencies

RUN apt update \
    && apt install -y --no-install-recommends python-pip python-catkin-tools \
    && rm -rf /var/lib/apt/lists/*

# Copy folder with only folders dependencies and package.xml as file (no other file)
COPY ${FOLDER_DEPENDENCIES_NAME} ${FOLDER_DEPENDENCIES_NAME}

# Install rosdep
RUN if [ "$(arch)" = "x86_64" ]; then export SKIPPED_ROS_KEYS="python-rpi.gpio"; fi \
    && apt update \
    && rosdep update && rosdep install --rosdistro ${ROS_DISTRO} --from-paths ${FOLDER_DEPENDENCIES_NAME} --ignore-src --default-yes --skip-keys "${SKIPPED_ROS_KEYS}" \
    && rm -rf /var/lib/apt/lists/*

COPY .docker/install_wiringpi.sh /opt/install_wiringpi.sh

RUN if [ "$(arch)" = "aarch64" ]; then /opt/install_wiringpi.sh; fi

ENV JOB_NUMBER=4

COPY .docker/docker_ros_entrypoint.sh /opt/docker_ros_entrypoint.sh

# Rosbridge suite
EXPOSE 9090

# Expose ros master port
EXPOSE 11311

ENTRYPOINT [ "/opt/docker_ros_entrypoint.sh" ]