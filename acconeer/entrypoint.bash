#!/bin/bash

source /opt/ros/noetic/setup.bash
source ws/devel/setup.bash
if [ "$RUN" = true ]; then
    echo "[ACCONEER] Launching Respiration Rate sensor"
    rosrun acconeer_radar_sensor respiration_radar_sensor.py --serial-port /dev/serial/by-id/usb-Silicon_Labs_Acconeer_XE125_R1DNL23122100268-if00-port0
else
    echo "[ACCONEER] RUN set to false, not launching Respiration Rate sensor"
fi
