#!/bin/bash
# 切换到「本地模式」：不连接机器人，8790/8796/8788/8789 等本地服务照常运行。
set -u
DIR="/opt/openhwlab/robot-viz"
cd "$DIR" || exit 1

touch "$DIR/robot_offline"
# 停掉所有会连机器人的服务
pkill -f '[j]oints_bridge\.py' 2>/dev/null || true
pkill -f '[c]amfeed\.py' 2>/dev/null || true
pkill -f '[c]amfeed_remote\.sh' 2>/dev/null || true
pkill -f '[g]raspproxy\.py' 2>/dev/null || true
sleep 0.5

bash "$DIR/serve.sh"
sleep 1.5
echo "== mode =="
curl -s -m 3 http://127.0.0.1:8796/joints | python3 -c 'import json,sys;d=json.load(sys.stdin);print({k:d.get(k) for k in ("status","offline","robot_online")})' 2>/dev/null || echo "bridge not ready"
echo "== 8790 =="
curl -s -o /dev/null -w '%{http_code}\n' -m 3 http://127.0.0.1:8790/robot_home.html
