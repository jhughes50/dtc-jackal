#!/bin/bash
#
ptp4l -f /etc/linuxptp/ptp4l.conf -i eno1 & phc2sys -a
