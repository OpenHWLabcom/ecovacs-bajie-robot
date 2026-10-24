#!/bin/bash
# 切回「真机模式」：清除本地模式标记，重连 adb，拉起关节桥/相机流。
set -u
DIR="/opt/openhwlab/robot-viz"
cd "$DIR" || exit 1
ADB="adb"
A="192.168.1.100:5555"

rm -f "$DIR/robot_offline"
pkill -f '[j]oints_bridge\.py' 2>/dev/null || true
pkill -f '[c]amfeed\.py' 2>/dev/null || true
pkill -f '[c]amfeed_remote\.sh' 2>/dev/null || true
pkill -f '[g]raspproxy\.py' 2>/dev/null || true
sleep 0.5

"$ADB" connect "${A%%:*}" >/dev/null 2>&1 || true
bash "$DIR/serve.sh"

echo "== 等待关节桥 =="
for i in $(seq 1 20); do
  out=$(curl -s -m 2 http://127.0.0.1:8796/joints 2>/dev/null)
  if echo "$out" | grep -q '"status"'; then
    echo "$out" | python3 -c 'import json,sys;d=json.load(sys.stdin);print({k:d.get(k) for k in ("status","offline","robot_online")})' 2>/dev/null
    exit 0
  fi
  sleep 1
done
echo "桥未就绪，查看 /tmp/robotviz-bridge.log"
