#!/bin/bash
# 八界 控制台：8790(HTTP 静态看板) + 8791/8792(HTTPS/HTTP 指挥服务) + 实体摇杆中继。
# 幂等启动，供 cron 保活。
# 本地模式：touch robot-viz/robot_offline（或 ./offline.sh）→ 不连机器人，页面/本地服务照常运行。
DIR="/opt/openhwlab/robot-viz"
cd "$DIR" || exit 1
ADB="adb"
A="192.168.1.100:5555"

OFFLINE=0
[ -f "$DIR/robot_offline" ] && OFFLINE=1
port_busy() { ss -ltn 2>/dev/null | awk '{print $4}' | grep -q ":$1$"; }

# ---------- 本地常驻：不依赖机器人 ----------
if ! pgrep -f "http_nocache\.py" >/dev/null 2>&1; then
  # 旧版静态服务器若占用 8790，先停掉（避免端口冲突）
  pkill -f "http\.server 8790" >/dev/null 2>&1
  sleep 0.3
  setsid -f python3 -u http_nocache.py >>/tmp/robotviz-8790.log 2>&1
fi
if ! pgrep -f "voice_server\.py" >/dev/null 2>&1; then
  setsid -f env ROBOT_HOST=127.0.0.1 ROBOT_DEV=192.168.1.100:5555 ROBOT_CLI_URL=http://127.0.0.1:17890/api/cli/exec \
    python3 -u voice_server.py >>/tmp/robotviz-8791.log 2>&1
fi
# 真机/本地模式控制服务(:8795)，始终运行
if ! pgrep -f "mode_svc\.py" >/dev/null 2>&1; then
  setsid -f python3 -u mode_svc.py >>/tmp/robotviz-mode-svc.log 2>&1
fi
# 实体摇杆（ESP32-P4 串口）中继：仅当串口在位且未运行时启动
if [ -e /dev/ttyACM0 ] && ! pgrep -f "joystick_relay\.py" >/dev/null 2>&1; then
  setsid -f python3 -u joystick_relay.py >>/tmp/robotviz-joy.log 2>&1
fi
# 无人机遥控屏摄像头(拍屏) MJPEG：仅当摄像头在位
if [ -e /dev/video0 ] && ! pgrep -f "cam_stream\.py" >/dev/null 2>&1; then
  setsid -f python3 -u cam_stream.py >>/tmp/robotviz-cam.log 2>&1
fi

if [ "$OFFLINE" = "1" ]; then
  echo "$(date '+%F %T') LOCAL MODE (robot_offline)" >>/tmp/robotviz-mode.log
  # 本地模式：桥只提供 /joints /health /mode，运动指令直接返回 offline
  if [ ! -f /tmp/robotviz.off ] && ! pgrep -f "joints_bridge\.py" >/dev/null 2>&1; then
    setsid -f env ROBOT_OFFLINE=1 python3 -u joints_bridge.py >>/tmp/robotviz-bridge.log 2>&1
  fi
else
  # ---------- 机器人链路保活 ----------
  # ① adb 保活：机器人(重)启动后 adb 若是断开/离线则自动重连。
  if ! timeout 9 "$ADB" -s "$A" shell echo ok >/dev/null 2>&1; then
    "$ADB" disconnect "$A" >/dev/null 2>&1
    r=$("$ADB" connect "$A" 2>&1 | tail -1)
    msg="reconnect: $r"
    last=$(tail -1 /tmp/robot-adb-keepalive.log 2>/dev/null | sed 's/^[0-9-]* [0-9:]* //')
    if [ "$msg" != "$last" ]; then
      echo "$(date '+%F %T') $msg" >> /tmp/robot-adb-keepalive.log
      tail -400 /tmp/robot-adb-keepalive.log > /tmp/robot-adb-keepalive.log.tmp 2>/dev/null && mv /tmp/robot-adb-keepalive.log.tmp /tmp/robot-adb-keepalive.log
    fi
  fi
  # ①b mission WS 走 adb forward
  "$ADB" -s "$A" forward tcp:17890 tcp:17890 >/dev/null 2>&1
  # ② 关节桥 joints_bridge(:8796)
  if [ ! -f /tmp/robotviz.off ] && ! pgrep -f "joints_bridge\.py" >/dev/null 2>&1; then
    setsid -f python3 -u joints_bridge.py >>/tmp/robotviz-bridge.log 2>&1
  fi
  # ④ 识别合成(:8799) + 爪子识别(:8800) 转播保活（touch /tmp/percfeed.off 可临时关）
  if [ ! -f /tmp/percfeed.off ] && [ ! -f /tmp/robotviz.off ]; then
    if ! port_busy 8799; then
      FEED_PORT=8799 FEED_SCRIPT=percd.py setsid -f python3 -u camfeed.py >>/tmp/robotviz-percfeed.log 2>&1
    fi
    if ! port_busy 8800; then
      FEED_PORT=8800 FEED_SCRIPT=percg.py setsid -f python3 -u camfeed.py >>/tmp/robotviz-gripfeed.log 2>&1
    fi
  fi
  # ⑤ 抓取日志代理(:8794)
  if [ ! -f /tmp/robotviz.off ] && ! pgrep -f "graspproxy\.py" >/dev/null 2>&1; then
    setsid -f python3 -u graspproxy.py >>/tmp/robotviz-graspproxy.log 2>&1
  fi
fi

# ---------- 本地服务（两种模式都运行）----------
# ⑥ 训练数据采集服务(:8789) 保活（定点头/通用 episode 采集，综合分析台数据源）
if [ ! -f /tmp/robotviz.off ] && ! pgrep -f "collector_svc\.py" >/dev/null 2>&1; then
  setsid -f python3 -u collector_svc.py >>/tmp/robotviz-collector.log 2>&1
fi

# ⑦ 爪头限位数据库服务(:8788) 保活（headfix_limits.html 数据源）
if [ ! -f /tmp/robotviz.off ] && ! pgrep -f "limits_svc\.py" >/dev/null 2>&1; then
  setsid -f python3 -u limits_svc.py >>/tmp/robotviz-limits.log 2>&1
fi
