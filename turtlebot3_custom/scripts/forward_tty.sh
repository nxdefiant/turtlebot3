#!/bin/sh
#
# Client:
# socat PTY,link=/dev/ttyS4,raw,echo=0,mode=777 TCP:tb3:10001

socat /dev/razorIMU,b57600 TCP4-LISTEN:10001,reuseaddr
