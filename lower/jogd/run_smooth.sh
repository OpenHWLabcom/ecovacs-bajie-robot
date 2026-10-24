#!/bin/bash
export PATH=/usr/bin:/bin:$PATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///factory/default.xml
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/Agi/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH=/home/Agi/install/lib:$LD_LIBRARY_PATH
pgrep -f petgripd.py >/dev/null 2>&1 || (cd /home/Agi/jogd && setsid nohup python3 /home/Agi/jogd/petgripd.py < /dev/null >> /tmp/petgripd.log 2>&1 &)
exec python3 -u /home/Agi/jogd/smoothtrack.py
