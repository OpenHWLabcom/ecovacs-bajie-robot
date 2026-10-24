#!/bin/bash
export PATH=/usr/bin:/bin:$PATH
pkill -f "${CAM_SCRIPT:-/home/Agi/jogd/camstream.py}" 2>/dev/null
sleep 1
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/Agi/install/setup.bash 2>/dev/null
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/Agi/jogd/dds_hi.xml
exec python3 "${CAM_SCRIPT:-/home/Agi/jogd/camstream.py}"
