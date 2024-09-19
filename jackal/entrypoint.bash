#!/bin/bash
source /opt/ros/noetic/setup.bash
source ws/devel/setup.bash
roslaunch jackal_launch jackal_hw.launch rgb:=$RGB ir:=$IR event:=$EVENT gps:=$GPS ouster:=$OUSTER
