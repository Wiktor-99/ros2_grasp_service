ARG ROS_DISTRO=rolling
FROM osrf/ros:$ROS_DISTRO-desktop
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive
RUN userdel $(getent passwd 1000 | cut -d: -f1)
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
    python3-pip pre-commit \
    ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-ros2-control \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    ros-$ROS_DISTRO-gripper-controllers \
    ros-$ROS_DISTRO-gz-ros2-control \
    && apt-get autoremove -y \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/$USERNAME/.bashrc \
    && echo "if [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then source /opt/ros/$ROS_DISTRO/setup.bash; fi" >> /home/$USERNAME/.bashrc
