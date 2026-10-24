#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/Agi/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///factory/default.xml
timeout 25 python3 /home/Agi/jogd/percd.py > /tmp/percd_test.bin 2>/tmp/percd_err.txt
echo BIN=$(stat -c%s /tmp/percd_test.bin)
echo ERRTAIL
tail -8 /tmp/percd_err.txt
