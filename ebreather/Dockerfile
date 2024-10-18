FROM nvidia/cuda:12.1.0-cudnn8-runtime-ubuntu20.04

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
COPY .bashrc . 

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

RUN sudo apt-get install -y python3-pip

RUN sudo adduser $USER dialout
RUN sudo adduser $USER tty 
RUN sudo adduser $USER plugdev

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

RUN sudo apt-get install -y software-properties-common \
 && sudo apt update \
 && sudo add-apt-repository ppa:inivation-ppa/inivation \
 && sudo apt-get update \
 && sudo apt-get install -y libcaer-dev

# setup environment
ARG NAME
ENV ROBOT=$NAME
RUN sudo chown $USER:$USER ~/.bashrc \
 && /bin/sh -c 'echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo "source ~/ws/devel/setup.bash" >> ~/.bashrc' \
 && echo 'export PS1="\[$(tput setaf 2; tput bold)\]\u\[$(tput setaf 7)\]@\[$(tput setaf 3)\]\h\[$(tput setaf 7)\]:\[$(tput setaf 4)\]\W\[$(tput setaf 7)\]$ \[$(tput sgr0)\]"' >> ~/.bashrc

RUN mkdir -p ws/src
COPY ./respiration_detection_ws/src/ ws/src
COPY ./entrypoint.bash entrypoint.bash
