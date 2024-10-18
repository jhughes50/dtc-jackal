#!/bin/bash

source /opt/ros/noetic/setup.bash
source ws/devel/setup.bash
if [ "$RUN" = true ]; then
    echo "[WHISPERER] Launching microphone listner"
    roslaunch whisperer whisperer.launch --wait timeout:=10
else
    echo "[WHISPERER] RUN set to false, not launching microphone listener"
fi
