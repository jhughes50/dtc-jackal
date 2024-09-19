#!/bin/bash

source /opt/ros/noetic/setup.bash
source ws/devel/setup.bash
if [ "$RUN" = true ]; then
    echo "[MTTS-CAN] Launching MTTS Network"
    roslaunch mtts_can_ros mtts.launch --wait
else
    echo "[MTTS-CAN] RUN set to false, not MTTS Network"
fi
