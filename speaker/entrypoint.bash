#!/bin/bash

source /opt/ros/noetic/setup.bash
source ws/devel/setup.bash
if [ "$RUN" = true ]; then
    echo "[SPEAKER] Launching speaker"
    roslaunch speakerer whisperer.launch --wait
else
    echo "[SPEAKER] RUN set to false, not launching speaker"
fi
