FROM ubuntu:20.04

RUN apt update \
 && apt install -y linuxptp ethtool tmux vim iproute2

RUN echo [eno1] >> /etc/linuxptp/ptp4l.conf

COPY entrypoint.bash entrypoint.bash
RUN chmod +x entrypoint.bash

ENTRYPOINT ["./entrypoint.bash"]
