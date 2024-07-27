###############################################################################
# 
# Copyright (C) 2024
# 
# Dockerfile for ROS2-foxy
#
# by Jo_09
# 
#########################################################################

#cpu architecture : x86_64 (ex. laptop, desktop)
#FROM ros:foxy

# 라즈베리파이와 같은 ARM64 아키텍처용 ROS 2 Foxy 이미지를 기반으로 합니다.
FROM arm64v8/ros:foxy

# APT 설정을 비대화형 모드로 설정하여 사용자 입력 대기 상태를 방지합니다.
ENV DEBIAN_FRONTEND=noninteractive

# 패키지 목록을 업데이트하고, 필요한 패키지들을 설치합니다.
RUN apt-get update \
    && apt-get install -y \
    nano \                      # 텍스트 편집기 nano
    x11-utils \                  # X11 애플리케이션 모음
    ros-foxy-desktop \          # ROS 2 Foxy 데스크탑 설치
    ros-foxy-rviz2 \            # ROS 2 RViz2 설치
    ros-foxy-twist-mux \        # ROS 2 Twist Mux 설치
    ros-foxy-xacro \            # ROS 2 Xacro 설치
    ros-foxy-ros2-control \     # ROS 2 컨트롤 라이브러리
    ros-foxy-ros2-controllers \ # ROS 2 컨트롤러 라이브러리
    ros-foxy-gazebo-ros2-control \ # Gazebo ROS 2 컨트롤 플러그인
    ros-foxy-slam-toolbox \     # SLAM 툴박스
    ros-foxy-navigation2 \      # 네비게이션2
    ros-foxy-nav2-bringup \     # 네비게이션2 브링업 패키지
    ros-foxy-cartographer \     # 카토그래퍼 패키지
    ros-foxy-cartographer-ros \ # 카토그래퍼 ROS 패키지
    joystick \                  # 조이스틱 유틸리티
    jstest-gtk \                # 조이스틱 테스트 유틸리티
    evtest \                    # 이벤트 테스트 유틸리티
    python3-pip \               # Python 3 pip 설치
    git \                       # Git 버전 관리 시스템 설치
    locales \                   # 로케일 설정 패키지
    tzdata \                    # 타임존 데이터 패키지
    qt5-default \               # QT5 기본 패키지
    && rm -rf /var/lib/apt/lists/* \ # APT 캐시 제거
    && apt-get clean             # APT 캐시 클리어

# 로케일 설정
RUN locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# 타임존 설정
RUN ln -fs /usr/share/zoneinfo/Asia/Seoul /etc/localtime \
    && dpkg-reconfigure --frontend noninteractive tzdata

# 사용자 추가
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

# 모든 패키지 업데이트 및 업그레이드
RUN sudo apt update && sudo apt upgrade -y

# Rosdep 업데이트 (이미 초기화된 경우 건너뛰기)
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rosdep init; fi \
    && rosdep update

# Source the ROS setup file
# ROS 환경 설정을 .bashrc 파일에 추가합니다.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
# DISPLAY 환경 변수를 설정하여 X11 디스플레이를 사용 가능하게 합니다.
RUN echo "export DISPLAY=:0" >> ~/.bashrc
# OpenGL을 소프트웨어 모드로 설정합니다.
RUN echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
RUN echo "source /desktop/AMR_Proj/Proj/dev-0716/src/agent_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /desktop/AMR_Proj/Proj/dev-0716/install/setup.bash" >> ~/.bashrc
RUN echo "sudo chmod 666 /dev/ttyUSB0 && sudo chmod 666 /dev/ttyUSB1 && sudo chmod 666 /dev/ttyACM0" >> ~/.bashrc

# NVIDIA Docker 설정을 추가하여 GPU를 사용할 수 있게 합니다.
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility

# NVIDIA Docker 런타임을 설치합니다.
RUN curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add - \
    && distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
    && curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list \
    && sudo apt-get update \
    && sudo apt-get install -y nvidia-docker2 \
    && sudo pkill -SIGHUP dockerd

# 컨테이너가 시작될 때 실행할 명령을 설정합니다.
ENTRYPOINT ["/bin/bash", "-c", "source ~/.bashrc && bash"]