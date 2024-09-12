#!/bin/bash

source /opt/ros/noetic/setup.bash
source ws/devel/setup.bash
rosrun acconeer_radar_sensor respiration_radar_sensor.py --serial-port /dev/serial/by-id/usb-Silicon_Labs_Acconeer_XE125_R1DNL23122100266-if00-port0
