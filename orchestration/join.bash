#!/bin/bash

xhost + 
docker exec -it --privileged -e DISPLAY=${DISPLAY} docker-jackal-orchestration-1 bash
