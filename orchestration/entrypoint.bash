#!/bin/bash

source /opt/ros/noetic/setup.bash
source ws/devel/setup.bash
if [ "$RUN" = true ]; then
    echo "[ORCHESTRATOR] Launching Orchestration"
    roslaunch gone orchestrator.launch --wait
else
    echo "[ORCHESTRATOR] RUN set to false, not launching orchestration"
fi
