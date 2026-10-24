#!/bin/bash
cat > /home/Agi/jogd/stow_play.py <<'PYEOF'
#!/usr/bin/env python3
# stow_play.py — 按学习到的进舱示范把臂收回归位（单发多点轨迹 + 收敛等待 + 一次慢速补正）
import json, sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

HOME = [0.0, 2.46, 3.256, -0.007, -2.543, 0.0]
NAMES = ["joint%d" % i for i in range(1, 7)]
TS = 1.1

pts = json.load(open('/home/Agi/stow_demo.json'))
N = len(pts)
t_raw = [p["tf"] for p in pts]
pos = [p["pos"] for p in pts]

rclpy.init(); node = Node("stowplay")
fb = {"p": None, "f": None}
def on_js(m):
    d = dict(zip(m.name, m.position)); v = [d.get(n) for n in NAMES]
    if all(x is not None for x in v): fb["p"] = [float(x) for x in v]
    if "arm_finger_joint1" in d: fb["f"] = float(d["arm_finger_joint1"])
node.create_subscription(JointState, "/joint_states", on_js, 50)

def spin(sec):
    t0 = time.time()
    while time.time() - t0 < sec: rclpy.spin_once(node, timeout_sec=0.1)
def cur_pose(sec=0.4):
    t0 = time.time()
    while fb["p"] is None and time.time() - t0 < 8: rclpy.spin_once(node, timeout_sec=0.1)
    last = fb["p"]; t0 = time.time()
    while time.time() - t0 < sec: rclpy.spin_once(node, timeout_sec=0.1); last = fb["p"]
    return last

spin(0.5)
f = fb["f"]
print("finger=%.3f, TS=%.2f" % (f if f is not None else -1, TS), flush=True)
if f is None or not (0.13 <= f <= 0.26):
    print("!! 夹子不在归位要求状态(0.13~0.26), 停止"); sys.exit(9)
ac = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
if not ac.wait_for_server(timeout_sec=10):
    print("NO ACTION SERVER"); sys.exit(2)

def send(gp, label):
    goal = FollowJointTrajectory.Goal(); traj = JointTrajectory(); traj.joint_names = NAMES
    for (t, p, v) in gp:
        q = JointTrajectoryPoint(); q.positions = p; q.velocities = v
        sec = int(t); q.time_from_start = Duration(sec=sec, nanosec=int((t - sec) * 1e9))
        traj.points.append(q)
    goal.trajectory = traj
    print("== %s: %d pts, %.2fs ==" % (label, len(gp), gp[-1][0]), flush=True)
    snd = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, snd, timeout_sec=8)
    gh = snd.result()
    if not (gh and gh.accepted):
        print("!! REJECTED"); return False
    res = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=int(gp[-1][0]) + 30)
    return True

gp = []
for k in range(N):
    t = t_raw[k] * TS
    if k == 0: v = [(pos[1][j] - pos[0][j]) / max(0.05, (t_raw[1] - t_raw[0]) * TS) for j in range(6)]
    elif k == N - 1: v = [(pos[N-1][j] - pos[N-2][j]) / max(0.05, (t_raw[N-1] - t_raw[N-2]) * TS) for j in range(6)]
    else: v = [(pos[k+1][j] - pos[k-1][j]) / max(0.05, (t_raw[k+1] - t_raw[k-1]) * TS) for j in range(6)]
    gp.append((t, pos[k], v))
if not send(gp, "收回归位·主轨迹"): sys.exit(3)

t0 = time.time(); end = cur_pose(0.1); err = max(abs(a - b) for a, b in zip(end, HOME))
while err > 0.02 and time.time() - t0 < 30:
    time.sleep(1.2); end = cur_pose(0.1); err = max(abs(a - b) for a, b in zip(end, HOME))
    print("   settling err=%.3f" % err, flush=True)

if err > 0.05:
    print("!! 主轨迹未到位(err=%.3f), 慢速补正" % err, flush=True)
    end2 = cur_pose(0.1); pts2 = []; K = 30
    for k in range(1, K + 1):
        d = float(k) / K; s = d * d * (3 - 2 * d); ds = 6 * d * (1 - d)
        p = [end2[i] + (HOME[i] - end2[i]) * s for i in range(6)]
        v = [(HOME[i] - end2[i]) / 2.5 * ds for i in range(6)]
        pts2.append((2.5 * d, p, v))
    send(pts2, "收回归位·慢速补正")
    t0 = time.time(); end = cur_pose(0.1); err = max(abs(a - b) for a, b in zip(end, HOME))
    while err > 0.02 and time.time() - t0 < 30:
        time.sleep(1.2); end = cur_pose(0.1); err = max(abs(a - b) for a, b in zip(end, HOME))
        print("   settling2 err=%.3f" % err, flush=True)

print("FINAL err=%.4f  end=%s" % (err, " ".join("%.3f" % x for x in end)), flush=True)
sys.exit(0 if err <= 0.05 else 4)
PYEOF
chmod +x /home/Agi/jogd/stow_play.py
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///factory/default.xml
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/Agi/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH=/home/Agi/install/lib:$LD_LIBRARY_PATH
rm -f /home/Agi/stow_play.out
setsid nohup bash -c 'python3 -u /home/Agi/jogd/stow_play.py > /home/Agi/stow_play.out 2>&1; echo EXIT=$? >> /home/Agi/stow_play.out' </dev/null &
echo "LAUNCHED"
