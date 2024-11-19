#!/bin/bash
source /opt/ros/noetic/setup.bash
source ws/devel/setup.bash
roslaunch sensors sensors.launch rgb:=$RGB ir:=$IR event:=$EVENT gps:=$GPS ouster:=$OUSTER zed:=$ZED
