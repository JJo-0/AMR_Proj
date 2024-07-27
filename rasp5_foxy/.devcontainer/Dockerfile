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
    nano \                      
    x11-apps \         
    ros-foxy-desktop \          
    ros-foxy-rviz2 \           
    ros-foxy-twist-mux \       
    ros-foxy-xacro \           
    ros-foxy-ros2-control \    
    ros-foxy-ros2-controllers \ 
    ros-foxy-gazebo-ros2-control \ 
    ros-foxy-slam-toolbox \     
    ros-foxy-navigation2 \      
    ros-foxy-nav2-bringup \     
    ros-foxy-cartographer \     
    ros-foxy-cartographer-ros \ 
    joystick \ 
    jstest-gtk \ 
    evtest \ 
    python3-pip \ 
    git \ 
    locales \                   
    tzdata \                    
    qt5-default \ 
    code \              
    && rm -rf /var/lib/apt/lists/* \ 
    && apt-get clean             

# Install language
RUN locale-gen en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/Asia/Seoul /etc/localtime \
    && dpkg-reconfigure --frontend noninteractive tzdata

# Add vscode user with same UID and GID as your host system
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y  \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME
RUN sudo apt update && sudo apt upgrade -y

# Rosdep update
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi \
    && rosdep update

RUN pip install pyserial

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "export DISPLAY=:0" >> ~/.bashrc
RUN echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.bashrc
RUN echo "source /desktop/AMR_Proj/Proj/dev-0716/src/agent_ws/install/setup.bash" >> ~/.bashrc
RUN echo "source /desktop/AMR_Proj/Proj/dev-0716/install/setup.bash" >> ~/.bashrc
RUN echo "sudo chmod 666 /dev/ttyUSB0 && sudo chmod 666 /dev/ttyUSB1 && sudo chmod 666 /dev/ttyACM0" >> ~/.bashrc
RUN echo "alias q='sudo chmod 666 /dev/ttyACM0'" >> ~/.bashrc
RUN echo "alias w='python3 /desktop/AMR_Proj/Proj/etc/touch_gui/std_GUI.py'" >> ~/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source ~/.bashrc && bash", "code"]