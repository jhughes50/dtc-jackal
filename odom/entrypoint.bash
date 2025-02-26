#!/bin/bash

source /opt/ros/noetic/setup.bash
source ws/devel/setup.bash
if [ "$RUN" = true ]; then
    echo "[ODOM] Launching Odometry"
    nohup roslaunch ouster_decoder ouster.launch sensor_hostname:=192.168.100.12 udp_dest:=192.168.100.1 lidar_mode:=1024x10 --wait &
    roslaunch rofl rofl_odom.launch --wait
else
    echo "[ODOM] RUN set to false"
fi
