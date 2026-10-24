#!/bin/bash
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///factory/default.xml
source /opt/ros/humble/setup.bash 2>/dev/null
pkill -f 'jogd/jogd[.]py' 2>/dev/null
pkill -f 'jogd/jogd_act[.]py' 2>/dev/null
sleep 0.6
exec python3 -u /home/Agi/jogd/jogd_act.py "$@"
