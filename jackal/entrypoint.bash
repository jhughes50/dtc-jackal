#!/bin/bash
source /opt/ros/noetic/setup.bash
source ws/devel/setup.bash
roscore &
roslaunch jackal_launch jackal_hw.launch --wait rgb:=$RGB ir:=$IR event:=$EVENT gps:=$GPS ouster:=$OUSTER
