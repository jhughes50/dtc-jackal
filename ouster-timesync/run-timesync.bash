#!/bin/bash
#

docker run --rm -it \
	--privileged \
	--network=host \
	-v "/dev:/dev" \
	ubuntu:20.04 \
	bash
