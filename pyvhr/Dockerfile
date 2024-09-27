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

RUN sudo apt-get install -y libhdf5-serial-dev
RUN sudo apt-get install -y python3-pip

COPY ./requirements.txt requirements.txt

RUN pip3 install -r requirements.txt
RUN python3 -m pip install tensorflow[and-cuda]
RUN pip3 install cupy-cuda12x
#RUN pip3 install --upgrade pip setuptool

#RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-keyring_1.1-1_all.deb \
# && sudo dpkg -i cuda-keyring_1.1-1_all.deb \
# && sudo apt-get update \
# && sudo apt-get -y install cuda-toolkit-12-6

RUN git clone https://github.com/rapidsai/cusignal.git \
    && cd cusignal \
    && git checkout v21.08.00

RUN pip3 install ./cusignal/python

COPY ./pyVHR pyVHR/
RUN cd pyVHR && pip3 install .

RUN sudo adduser $USER dialout
RUN sudo adduser $USER tty 
RUN sudo adduser $USER plugdev

# SETUP
# setup environment
ARG NAME
ENV ROBOT=$NAME
RUN sudo chown $USER:$USER ~/.bashrc \
 && /bin/sh -c 'echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo "source ~/ws/devel/setup.bash" >> ~/.bashrc' \
 && echo 'export PS1="\[$(tput setaf 2; tput bold)\]\u\[$(tput setaf 7)\]@\[$(tput setaf 3)\]\h\[$(tput setaf 7)\]:\[$(tput setaf 4)\]\W\[$(tput setaf 7)\]$ \[$(tput sgr0)\]"' >> ~/.bashrc

RUN mkdir -p ws/src
COPY ./rosvhr ./ws/src/rosvhr
COPY ./entrypoint.bash entrypoint.bash

ENV RUN=true

RUN cd ~/ws \
 && catkin config --extend /opt/ros/noetic \
 && catkin build --no-status -DCMAKE_BUILD_TYPE=Release
