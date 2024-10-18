#!/bin/bash

source /opt/ros/noetic/setup.bash
source ws/devel/setup.bash
if [ "$RUN" = true ]; then
    echo "[PYVHR] Launching pyVHR Heart Rate Detection"
    rosrun pyvhr pyvhr_node.py
else
    echo "[PYVHR] RUN set to false, not launching pyVHR heart rate detection"
fi
