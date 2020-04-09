#!/bin/bash

# see https://medium.com/@spe_/debugging-c-c-programs-remotely-using-visual-studio-code-and-gdbserver-559d3434fb78

# synchronize the current folder with the remote pi (must be at that IP address)
rsync -avzp --exclude='docs' --exclude='.git' --del ./ pi@192.168.1.15:~/ardupilot