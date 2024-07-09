###############################################################################
# 
# Copyright (C) 2024
# 
# Dockerfile for ROS2-Humble
#
# by Jo_09
# 
#########################################################################

#cpu architecture : x86_64 (ex. laptop, desktop)
FROM ros:humble

#cpu architecture : aaarch64, arm64 (ex raspberry pi 5, Jetson nano)
#FROM arm64v8/ros:humble

# APT 설정을 non-interactive 모드로 변경하여 사용자 입력 대기 상태를 방지합니다.
ENV DEBIAN_FRONTEND=noninteractive

# Example of installing programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    x11-apps \
    ros-humble-desktop \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-gazebo-ros2-control \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    joystick jstest-gtk evtest \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Install language
RUN apt-get update \
    && apt-get install -y \
    locales \
    && locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/Asia/Seoul /etc/localtime \
    && export DEBIAN_FRONTEND=noninteractive \
    && apt-get update \
    && apt-get install -y tzdata \
    && dpkg-reconfigure --frontend noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Raspberry pi 5 does not have a GPU, so we need to force software rendering
ENV LIBGL_ALWAYS_SOFTWARE=1
