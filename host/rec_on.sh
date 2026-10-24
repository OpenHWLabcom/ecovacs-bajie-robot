#!/bin/bash
cat > /home/Agi/jogd/samp3.py <<'PYEOF'
#!/usr/bin/env python3
# samp3.py — 100Hz 全关节(含夹爪/升降/云台)记录器
import rclpy, time, sys
from rclpy.node import Node
from sensor_msgs.msg import JointState
OUT, DUR = sys.argv[1], float(sys.argv[2])
KEYS = ['joint1','joint2','joint3','joint4','joint5','joint6','arm_finger_joint1','lift_joint','pitch_joint']
rclpy.init(); n = Node('samp3'); f = open(OUT, 'a', buffering=1)
last = [0.0]
def cb(m):
    t = time.time()
    if t - last[0] < 0.009: return
    last[0] = t
    d = dict(zip(m.name, m.position))
    f.write("S %.3f %s\n" % (t, ' '.join('%.4f' % d.get(k, float('nan')) for k in KEYS)))
n.create_subscription(JointState, '/joint_states', cb, 100)
t0 = time.time()
while time.time() - t0 < DUR:
    rclpy.spin_once(n, timeout_sec=0.5)
f.close()
PYEOF
pkill -f 'jogd/samp3[.]py' 2>/dev/null; sleep 0.3
setsid nohup python3 -u /home/Agi/jogd/samp3.py /home/Agi/cap_learn.csv 7200 >/tmp/samp3.log 2>&1 </dev/null &
sleep 2
ps -eo pid,cmd | grep 'samp3' | grep -v grep && echo "samp3 OK" || echo "samp3 FAIL"
pkill -f 'tail -n0 -F /userdata/logs' 2>/dev/null; sleep 0.3
setsid nohup bash -c 'tail -n0 -F /userdata/logs/ros2_control_node/rosout.log | grep --line-buffered -aE "point\[|positions:|velocities:|new trajectory accepted|Received new action goal" >> /home/Agi/traj_cap2.log' >/dev/null 2>&1 </dev/null &
sleep 1
ps -eo pid,cmd | grep 'traj_cap2\|tail -n0' | grep -v grep | head -3 && echo "traj_cap2 OK" || echo "traj_cap2 FAIL"
echo "=== rosout 行数/大小 ==="
wc -l /userdata/logs/ros2_control_node/rosout.log; ls -l /userdata/logs/ros2_control_node/rosout.log | awk '{print $5}'
echo "=== 19:55:09 accepted 行号 ==="
grep -n "19:55:09.244" /userdata/logs/ros2_control_node/rosout.log | head -3
echo "=== 该行前后窗口 (看 dump 放置位置) ==="
LN=$(grep -n "19:55:09.244" /userdata/logs/ros2_control_node/rosout.log | head -1 | cut -d: -f1)
echo "center line: $LN"
sed -n "$((LN-20)),$((LN+60))p" /userdata/logs/ros2_control_node/rosout.log | head -85
