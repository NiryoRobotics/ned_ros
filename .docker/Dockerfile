ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}-perception AS BUILD_DEPENDENCIES

RUN apt update \
    && apt install -y --no-install-recommends python3-pip python3-catkin-tools cpio git ros-${ROS_DISTRO}-rosdoc-lite\
    && rm -rf /var/lib/apt/lists/*

RUN find . -type f -name "package.xml" | cpio -pdm docker_ros_folder_dependencies

# Install rosdep
RUN apt update \
    && rosdep update  \
    && rosdep install --rosdistro ${ROS_DISTRO} --from-paths docker_ros_folder_dependencies --ignore-src --default-yes --skip-keys="python3-rpi.gpio python-rpi.gpio" \
    && rm -rf /var/lib/apt/lists/*

# Install additionnal dependencies
# ros-babel-fish is a foxglove dependecy which is not detected by rosdep for some reasons
RUN apt update \
    && apt install -y --no-install-recommends ros-${ROS_DISTRO}-rosdoc-lite curl ros-${ROS_DISTRO}-ros-babel-fish/focal\
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
