###############################################################################
# 
# Copyright (C) 2024
# 
# Dockerfile for ROS2-foxy
#
# by Jo_09
# 
#########################################################################
#cpu architecture : aaarch64, arm64 (ex raspberry pi 5, Jetson nano)
FROM kasmweb/debian-bookworm-desktop:develop

# APT 설정을 non-interactive 모드로 변경하여 사용자 입력 대기 상태를 방지합니다.
ENV DEBIAN_FRONTEND=noninteractive

# apt-get 관련 잠재적 권한 문제 해결
RUN mkdir -p /var/lib/apt/lists/partial
RUN chmod -R 755 /var/lib/apt/lists

# Example of installing programs
RUN apt-get update \
    && apt-get install -y \
    nano \
    x11-apps \
    git \ 
    colcon-common-extensions \
    vcstool \
    wget \
    libx11-dev \
    libxrandr-dev \
    libasio-dev \
    libtinyxml2-dev \
    python3-pip \
    python3-rosdep2 \
    python3-vcstool \
    python3-flake8 \
    python3-flake8-docstrings \
    python3-flake8-blind-except \
    python3-flake8-builtins \
    python3-flake8-class-newline \
    python3-flake8-comprehensions \
    python3-flake8-deprecated \
    python3-flake8-import-order \
    python3-flake8-quotes \
    python3-pytest-repeat \
    python3-pytest-cov \
    python3-pytest-rerunfailures \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

# Install language
RUN apt-get update && apt-get install -y locales tzdata \
    && echo "en_US.UTF-8 UTF-8" > /etc/locale.gen \
    && locale-gen en_US.UTF-8 \
    && dpkg-reconfigure locales \
    && echo "Asia/Seoul" > /etc/timezone \
    && dpkg-reconfigure -f noninteractive tzdata \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# rosdep 초기화 및 업데이트
RUN rosdep init && rosdep update

# ROS 2 Humble 소스 코드를 다운로드
RUN mkdir -p /root/ros2_humble/src
WORKDIR /root/ros2_humble
RUN vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

# 의존성 설치
RUN rosdep install --from-paths src --ignore-src --rosdistro humble -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers python3-vcstool"

# ROS 2 빌드
RUN colcon build --symlink-install

# 환경 설정
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/$ROS_DISTRO
ENV PATH=$ROS_ROOT/bin:$PATH
ENV LD_LIBRARY_PATH=$ROS_ROOT/lib:$LD_LIBRARY_PATH
ENV PYTHONPATH=$ROS_ROOT/lib/python3.8/site-packages:$PYTHONPATH

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# ROS 환경 설정 파일 소싱
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Raspberry Pi 5 GPU 부재 관련 설정
ENV LIBGL_ALWAYS_SOFTWARE=1

