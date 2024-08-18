FROM osrf/ros:iron-desktop
ARG ROS_DISTRO=iron
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV DEBIAN_FRONTEND=dialog
ENV SHELL /bin/bash

RUN apt-get update && sudo apt-get install -y lsb-release wget gnupg

RUN sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update --fix-missing \
    && apt-get -y install \
    gz-harmonic \
    ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros-gzharmonic-sim \
    ros-$ROS_DISTRO-ros-gzharmonic-bridge \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-gripper-controllers \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

ENV GZ_VERSION=harmonic
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    mkdir -p /gz_ros2_control_ws/src && mkdir -p /gz_ros2_control_ws/src && \
    cd /gz_ros2_control_ws/src && \
    git clone --branch iron https://github.com/ros-controls/gz_ros2_control && \
    cd /gz_ros2_control_ws && \
    colcon build

ENV DEBIAN_FRONTEND=dialog

ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc
RUN pip3 install pre-commit
