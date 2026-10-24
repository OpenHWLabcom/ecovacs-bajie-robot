#!/bin/bash
# 采样器启动包装 (source: robot-viz/samp_run.sh，部署于 /home/Agi/jogd/samp_run.sh)
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///factory/default.xml
source /opt/ros/humble/setup.bash 2>/dev/null
exec python3 -u /home/Agi/jogd/samp.py "$@"
