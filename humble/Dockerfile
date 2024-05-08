###############################################################################
# 
# Copyright (C) 2024
# 
# Dockerfile for ROS2-Humble
#
# by Jo_09
# 
#########################################################################

FROM arm64v8/ros:humble

# Example of installing programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    x11-apps \
    ros-humble-desktop \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown -R $USERNAME:$USERNAME /home/$USERNAME/.config

# copy config files
#COPY config/ /etc/ros/ros2/

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

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

# Raspberry pi 5 does not have a GPU, so we need to force software rendering
ENV LIBGL_ALWAYS_SOFTWARE=1

# Copy the entrypoint and bashrc scripts so we have
# our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc

