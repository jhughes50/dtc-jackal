#!/bin/bash

xhost +
docker exec -it --privileged -e DISPLAY=${DISPLAY} dtc-jackal-jackal-sensors-1 bash
xhost -
