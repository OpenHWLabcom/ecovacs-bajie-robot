#!/bin/bash
# 八界下板 jogd 启动包装 (source: robot-viz/jogd_run.sh，部署于 /home/Agi/jogd/run.sh)
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///factory/default.xml
source /opt/ros/humble/setup.bash 2>/dev/null
# 单实例：清掉可能残留的旧 jogd（桥重启时旧会话可能被孤儿化）
pkill -f 'jogd/jogd[.]py' 2>/dev/null
sleep 0.6
exec python3 -u /home/Agi/jogd/jogd.py "$@"
