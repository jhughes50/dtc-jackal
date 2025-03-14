#!/bin/bash

set -eo pipefail

# Check that the current user has UID 1000.
if [ $(id -u) -ne 1000 ]
then
  echo "ERROR: This script must be run with UID and GID of 1000."
  echo "       Current UID: $(id -u), current GID: $(id -g)"
  exit 1
fi

# Make sure processes in the container can connect to the x server
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    touch $XAUTH
fi
xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
if [ -n "$xauth_list" ]
then
  echo "$xauth_list" | xauth -f $XAUTH nmerge -
fi
chmod a+r $XAUTH

docker run -it --rm --gpus all \
    --network=host \
    -u $UID \
    --privileged \
    -e "TERM=xterm-256color" \
    -v "/dev:/dev" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "./data:/home/`whoami`/data" \
    -e PULSE_SERVER=unix:/run/user/1000/pulse/native \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "tmux.conf:/home/`whoami`/.tmux.conf" \
    --add-host dtc:192.168.8.100 \
    --add-host dtc:127.0.0.1 \
    --security-opt seccomp=unconfined \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -v "./launch/camera.launch:/home/`whoami`/ws/src/flir_camera_driver/spinnaker_camera_driver/launch/camera.launch" \
    -v "./config/boson.yaml:/home/`whoami`/ws/src/eeyoreROS/config/boson.yaml" \
    -v "./config/zed_f9p.yaml:/home/`whoami`/ws/src/ublox/ublox_gps/config/zed_f9p.yaml" \
    --name dtc-jackal-$(hostname)-sensors \
    dtc-jackal-$(hostname):sensors \
    bash
