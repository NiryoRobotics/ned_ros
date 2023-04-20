ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}-perception AS BUILD_DEPENDENCIES

RUN apt update \
    && apt install -y --no-install-recommends python3-pip python3-catkin-tools cpio\
    && rm -rf /var/lib/apt/lists/*

RUN find . -type f -name "package.xml" | cpio -pdm docker_ros_folder_dependencies

# Install rosdep
RUN if [ "$(arch)" = "x86_64" ]; then export SKIPPED_ROS_KEYS="python3-rpi.gpio python-rpi.gpio"; fi \
    && apt update \
    && rosdep update  \
    && rosdep install --rosdistro ${ROS_DISTRO} --from-paths docker_ros_folder_dependencies --ignore-src --default-yes --skip-keys="${SKIPPED_ROS_KEYS}" \
    && rm -rf /var/lib/apt/lists/*

# Install additionnal dependencies
RUN apt update \
    && apt install -y --no-install-recommends ros-${ROS_DISTRO}-rosdoc-lite curl\
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
