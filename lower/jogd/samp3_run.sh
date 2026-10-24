#!/bin/bash
export PATH=/usr/bin:/bin:$PATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///factory/default.xml
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/Agi/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH=/home/Agi/install/lib:$LD_LIBRARY_PATH
exec python3 -u /home/Agi/jogd/samp3.py "$1" "$2"
