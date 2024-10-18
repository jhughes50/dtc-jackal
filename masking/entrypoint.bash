#!/bin/bash

source /opt/ros/noetic/setup.bash
source ws/devel/setup.bash
if [ "$RUN" = true ]; then
    echo "[MASKER] Launching person masking node"
    roslaunch yolov8_ros mask.launch --wait
else
    echo "[MASKER] RUN set to false, not launching masking node"
fi
