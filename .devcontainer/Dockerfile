FROM ros:noetic-perception-focal

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create and switch to user
RUN groupadd -g $USER_GID $USERNAME \
    && useradd -lm -u $USER_UID -g $USERNAME -s /bin/bash $USERNAME \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Create workspace so that user own this directory
RUN mkdir -p /home/$USERNAME/catkin_ws/src
WORKDIR /home/$USERNAME/catkin_ws

# Change ownership
RUN sudo chown -R $USERNAME:$USERNAME /home/$USERNAME/catkin_ws

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git 

# Install other packages
RUN sudo apt update && sudo apt install -y \
    ros-noetic-xacro \
    python3-catkin-tools \
    ros-noetic-plotjuggler-ros \
    ros-noetic-ros-babel-fish \
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu \
    clang \
    clang-format \
    rsync \
    x11-apps \
    libx11-dev \
    bash-completion 
    bash-completion \
    python3-pip

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

