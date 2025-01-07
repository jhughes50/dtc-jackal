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

RUN mkdir -p ~/cpp_deps
RUN cd ~/cpp_deps \
  && git clone --depth 1 --branch v0.6.0 https://github.com/google/glog.git \
  && cd glog \
  && cmake -S . -B build -G "Unix Makefiles" -DCMAKE_CXX_STANDARD=17 \
  && cmake --build build \
  && sudo cmake --build build --target install
RUN cd ~/cpp_deps \
  && git clone --depth 1 --branch 8.1.0 https://github.com/fmtlib/fmt.git \
  && cd fmt && cmake -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE -DCMAKE_CXX_STANDARD=17 -DFMT_TEST=False \
  && sudo make install
RUN cd ~/cpp_deps \
  && git clone --depth 1 --branch 20220623.0 https://github.com/abseil/abseil-cpp.git \
  && cd abseil-cpp \
  && mkdir build \
  && cd build \
  && cmake -DABSL_BUILD_TESTING=OFF -DCMAKE_CXX_STANDARD=17 -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_POSITION_INDEPENDENT_CODE=ON .. \
  && sudo sudo cmake --build . --target install
RUN cd ~/cpp_deps \
  && git clone https://github.com/strasdat/Sophus.git \
  && cd Sophus \
  && git checkout 785fef3 \
  && mkdir build \
  && cd build \
  && cmake -DBUILD_SOPHUS_TESTS=OFF -DBUILD_SOPHUS_EXAMPLES=OFF -DCMAKE_CXX_STANDARD=17 .. \
  && sudo make install -j8
RUN cd ~/cpp_deps \
  && git clone https://github.com/google/benchmark.git \
  && cd benchmark && cmake -E make_directory "build" \
  && cmake -E chdir "build" cmake -DBENCHMARK_DOWNLOAD_DEPENDENCIES=on -DCMAKE_BUILD_TYPE=Release ../ \
  && sudo cmake --build "build" --config Release --target install
RUN cd ~/cpp_deps \
  && git clone --recursive https://github.com/stevenlovegrove/Pangolin.git \
  && cd Pangolin \
  && git checkout v0.9.1 \
  && sudo apt install -y $(./scripts/install_prerequisites.sh required --list) \
  && cmake -B build \
  && sudo cmake --build build --config Release --target install

RUN sudo apt-get install -y ros-noetic-pcl-ros
RUN sudo apt-get install -y build-essential \
    libeigen3-dev \
    libjsoncpp-dev \
    libspdlog-dev \
    libcurl4-openssl-dev

RUN mkdir -p ~/ws/src/
RUN cd ws/src \
 && git clone -b refactor/v0.10 https://github.com/KumarRobotics/ouster_decoder \
 && git clone --recurse-submodules https://github.com/ouster-lidar/ouster-ros \
 && git clone https://github.com/versatran01/rofl-beta

RUN sudo chown $USER:$USER ~/.bashrc \
 && /bin/sh -c 'echo ". /opt/ros/noetic/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo "source ~/ws/devel/setup.bash" >> ~/.bashrc' \
 && echo 'export PS1="\[$(tput setaf 2; tput bold)\]\u\[$(tput setaf 7)\]@\[$(tput setaf 3)\]\h\[$(tput setaf 7)\]:\[$(tput setaf 4)\]\W\[$(tput setaf 7)\]$ \[$(tput sgr0)\]"' >> ~/.bashrc

COPY ./entrypoint.bash entrypoint.bash

RUN cd ~/ws \
 && catkin config --extend /opt/ros/noetic \
 && catkin build --no-status -DCMAKE_BUILD_TYPE=Release
