#!/bin/bash

source /opt/ros/noetic/setup.bash
source ws/devel/setup.bash
roslaunch whisperer whisperer.launch --wait timeout:=10
