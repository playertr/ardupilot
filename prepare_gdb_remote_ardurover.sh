#!/bin/bash

# see https://medium.com/@spe_/debugging-c-c-programs-remotely-using-visual-studio-code-and-gdbserver-559d3434fb78

# synchronize the current folder with the remote pi (must be at that IP address)
#rsync -avzp --exclude='docs' --exclude='.git' --del ./ pi@192.168.1.15:~/ardupilot

# Kill gdbserver if it's running
# Kill gdbserver if it's running
# Compile myprogram and launch gdbserver, listening on port 9091
ssh -L9091:localhost:9091 pi@realtimepi.local bash -c "'
  sudo killall gdbserver &> /dev/null
  sudo gdbserver :9091 /home/pi/usr/bin/ardurover -A udp:192.168.1.5:14550 -B /dev/ttyAMA0'"