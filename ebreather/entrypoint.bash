#!/bin/bash

source /opt/ros/noetic/setup.bash
source ws/devel/setup.bash
if [ "$RUN" = true ]; then
    echo "[EBREATHER] Luanching Event Respiration Rate Detection"
else
    echo "[EBREATHER] RUN set to false, not launching event respiration rate de"
fi
