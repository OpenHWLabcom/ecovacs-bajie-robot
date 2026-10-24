#!/bin/bash
export PATH=/usr/bin:/bin:$PATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/Agi/jogd/dds_hi.xml
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/Agi/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH=/home/Agi/install/lib:$LD_LIBRARY_PATH
# petgripd 自动启动已停用(2026-10-17 防振动/防占DDS)
exec python3 -u /home/Agi/jogd/smoothtrack.py
