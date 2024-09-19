#!/bin/bash

xhost +
docker exec -it --privileged -e DISPLAY=${DISPLAY} docker-jackal-base-1 bash
xhost -
