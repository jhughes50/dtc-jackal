FROM nvidia/cuda:12.1.0-cudnn8-devel-ubuntu20.04

RUN apt-get update \
 && export TZ="America/New_York" \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y locales \
 && ln -fs "/usr/share/zoneinfo/$TZ" /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && apt-get clean

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
 lsb-release \
 libgl1-mesa-dri

ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales

# install the basics
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
 vim \
 tmux \
 cmake \
 gcc \
 g++ \
 git \
 build-essential \
 sudo \
 wget \
 curl \
 zip \
 unzip

# add a user
ARG user_id
ARG USER dtc
RUN useradd -U --uid ${user_id} -ms /bin/bash $USER \
 && echo "$USER:$USER" | chpasswd \
 && adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

# Set locales
RUN sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    locale-gen
ENV LC_ALL en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en

USER $USER
WORKDIR /home/$USER
RUN sudo adduser $USER dialout
RUN sudo adduser $USER tty 
RUN sudo adduser $USER plugdev

# Install ROS Noetic
RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \ && sudo /bin/sh -c 'curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -' \
 && sudo apt-get update \
 && sudo apt-get install -y \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    ros-noetic-desktop-full

RUN sudo rosdep init \
 && sudo apt-get clean

RUN rosdep update

COPY .bashrc .

# Install ROS dependencies
RUN sudo apt-get install -y \
    ros-noetic-nmea-msgs \
    ros-noetic-nmea-navsat-driver \
    ros-noetic-rosserial \
    ros-noetic-rosserial-server \
    ros-noetic-teleop-twist-joy \
    ros-noetic-vision-msgs \
    ros-noetic-rosbridge-server \
    ros-noetic-image-exposure-msgs \
    ros-noetic-wfov-camera-msgs

# DRIVER DEPENDENCIES
# Install the metavision SDK
RUN sudo apt-get update \
 && sudo sh -c 'echo "deb [arch=amd64 trusted=yes] https://apt.prophesee.ai/dists/public/baiTh5si/ubuntu focal sdk" > /etc/apt/sources.list.d/metavision.list' \
 && sudo sh -c 'echo "deb [trusted=yes] http://ppa.launchpad.net/s-schmeisser/ogre-1.12/ubuntu focal main" >> /etc/apt/sources.list.d/metavision.list' \
 && sudo apt-get update \
 && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
    libcanberra-gtk-module \
    mesa-utils \
    python3.8-dev \
    libboost-program-options-dev \
    libeigen3-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    metavision-sdk \
    metavision-sdk-samples \
    metavision-sdk-advanced-samples \
    metavision-sdk-cv-samples \
    libogre-1.12-dev \
    ffmpeg \
    libx264-dev

# Dependencies required by metavision
RUN sudo apt-get install -y python3-pip python3-dbg python3-venv
RUN python3 -m pip install "opencv-python==4.5.5.64" "sk-video==1.1.10" "fire==0.4.0" "numpy==1.23.4" pandas scipy
RUN sudo apt-get update \
 && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-noetic-pybind11-catkin \
    clang-tidy

# Spinnaker SDK
COPY install install/
RUN cd ~/install/spinnaker-3.2.0.57-amd64 \
 && sudo apt-get update \
 && yes yes | DEBIAN_FRONTEND=noninteractive sudo dpkg -i libgentl_*.deb \
 && sudo dpkg -i libspinnaker_*.deb \
 && sudo dpkg -i libspinnaker-dev_*.deb \
 && sudo dpkg -i libspinnaker-c_*.deb \
 && sudo dpkg -i libspinnaker-c-dev_*.deb \
 && sudo dpkg -i libspinvideo_*.deb \
 && sudo dpkg -i libspinvideo-dev_*.deb \
 && sudo dpkg -i libspinvideo-c_*.deb \
 && sudo dpkg -i libspinvideo-c-dev_*.deb \
 && sudo apt-get install -y ./spinview-qt_*.deb \
 && sudo dpkg -i spinview-qt-dev_*.deb \
 && sudo dpkg -i spinupdate_*.deb \
 && sudo dpkg -i spinupdate-dev_*.deb \
 && sudo dpkg -i spinnaker_*.deb \
 && sudo dpkg -i spinnaker-doc_*.deb \
 && sudo bash configure_usbfs.sh

# Install Boson-SDK
RUN mkdir ~/Repos \
 && cd . \
 && cd . \
 && git clone https://github.com/jhughes50/boson-sdk \
 && cd boson-sdk \
 && sudo bash buildAndInstall.sh

WORKDIR /home/$USER

# install spinnaker ros driver
RUN mkdir -p ws/src \
 && cd ws/src \
 && git clone https://github.com/ros-drivers/flir_camera_driver \
 && cd flir_camera_driver \
 && git switch noetic-devel 

# install boson ros drivers
RUN cd ws/src \
 && git clone https://github.com/jhughes50/eeyore \
 && git clone https://github.com/jhughes50/eeyoreROS

# install ublox driver
RUN cd ws/src \
 && git clone https://github.com/KumarRobotics/ublox \
 && git clone https://github.com/tilk/rtcm_msgs \
 && git clone https://github.com/LORD-MicroStrain/ntrip_client

# install event camera ros drivers
SHELL ["/bin/bash", "-c"]
RUN cd ws/src \
 && source /opt/ros/noetic/setup.bash \
 && git clone https://github.com/berndpfrommer/metavision_ros_driver.git \
 && cd metavision_ros_driver \
 && git checkout ab2932035200eccefcc63d943643a9020e836c61 \
 && cd .. \
 && git clone https://github.com/berndpfrommer/event_array_py.git \
 && cd event_array_py \
 && git checkout 3c8578f66610493a11c6208376a4a22870162c37 \
 && cd .. \
 && git clone https://github.com/k-chaney/simple_image_recon.git \
 && cd simple_image_recon \
 && git checkout 3ba9d1aa964e6d493d9ec8d52e8c71ea3e8f4976 \
 && cd .. \
 && git clone https://github.com/berndpfrommer/simple_image_recon_lib.git \
 && cd simple_image_recon_lib \
 && git checkout 328d723f2941131b317471d97a534aaba5438003 \
 && cd .. \
 && git clone https://github.com/ros-event-camera/event_camera_msgs\
 && cd event_camera_msgs \
 && git checkout c4dc0012f230b8acc0887b50e7e08ef6dedd14fe \
 && cd .. \
 && git clone https://github.com/ros-event-camera/event_camera_codecs \
 && cd event_camera_codecs \
 && git checkout 4be8d710d537b678e5aae67cf5b9b807f0d92149 \
 && cd ../.. \
 && wstool init src src/metavision_ros_driver/metavision_ros_driver.rosinstall \
 && wstool merge -t src src/event_array_py/event_array_py.rosinstall \
 && wstool update -t src

# ZED Install
#RUN sudo apt update && sudo apt install -y libnvidia-encode-545 libnvidia-decode-545 zstd
#RUN wget https://download.stereolabs.com/zedsdk/4.2/cu12/ubuntu20 && chmod +x ubuntu20 && ./ubuntu20 -- silent
#RUN cd ws/src/ && git clone --recursive https://github.com/KumarRobotics/zed-ros-wrapper.git
#RUN cd ws/ && rosdep install --from-paths src --ignore-src -r -y  --rosdistro=${ROS_DISTRO}

# ouster drivers
RUN sudo apt install -y libspdlog-dev 
RUN cd ws/src \
 && git clone https://github.com/KumarRobotics/ouster_decoder \
 && cd ouster_decoder \
 && git switch refactor/v0.10 \
 && cd .. \
 && git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros

# Audio
RUN sudo apt update \
 && sudo apt-get install -y --fix-missing ros-noetic-audio-common
RUN sudo adduser $USER audio
# blackfly clean up
RUN sudo groupadd flirimaging
RUN sudo adduser $USER flirimaging

# Sensor startup
COPY sensors/ ws/src/sensors

RUN sudo apt-get install -y libogre-1.9-dev ros-noetic-rviz

# BUILD
# build the ROS workspace 

RUN cd ~/ws \
 && sudo chown $USER:$USER ~/.config \
 && catkin config --extend /opt/ros/noetic --skiplist laser_gazebo_plugins_organized \
 && catkin build --no-status -DCMAKE_BUILD_TYPE=Release

COPY .bashrc .bashrc
COPY entrypoint.bash entrypoint.bash
# SETUP
# setup environment
ARG NAME
ENV ROBOT=$NAME

ENV OUSTER=false
ENV RGB=false
ENV IR=false
ENV EVENT=false
ENV GPS=false
ENV ZED=false
ENV MICROPHONE=false

RUN sudo chown $USER:$USER ~/.bashrc \
 && /bin/sh -c 'echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo "source ~/ws/devel/setup.bash" >> ~/.bashrc' \
 && echo 'export PS1="\[$(tput setaf 2; tput bold)\]\u\[$(tput setaf 7)\]@\[$(tput setaf 3)\]\h\[$(tput setaf 7)\]:\[$(tput setaf 4)\]\W\[$(tput setaf 7)\]$ \[$(tput sgr0)\]"' >> ~/.bashrc
