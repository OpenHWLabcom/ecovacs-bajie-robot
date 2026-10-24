#!/bin/bash
export PATH=/usr/bin:/bin
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/Agi/install/setup.bash 2>/dev/null
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/Agi/jogd/dds_hi.xml
python3 -u /home/Agi/jogd/box3d.py "$@"
