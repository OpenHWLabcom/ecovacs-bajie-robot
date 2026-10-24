#!/bin/bash
cat > /home/Agi/jogd/seqmove4.py <<'PYEOF'
#!/usr/bin/env python3
# seqmove4.py v3 — 归位→开箱→维护→归位; 归位段分3小段爬行 + 夹子门限 + 收敛等待 + 摆动救援
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

NAMES = ["joint%d" % i for i in range(1, 7)]
P_HOME  = [0.0, 2.46, 3.256, -0.007, -2.543, 0.0]
P_OPEN  = [0.0, 1.766, 2.887, 0.0, -0.451, 0.0]
P_MAINT = [0.045, 2.452, 2.041, -0.023, 0.232, 0.0]

rclpy.init(); node = Node("seqmove4")
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

def lerp(a, b, s): return [a[i] + (b[i] - a[i]) * s for i in range(6)]

ac = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
if not ac.wait_for_server(timeout_sec=10):
    print("NO ACTION SERVER"); sys.exit(2)

def send_seg(name, tgt, dur, wait_max=12.0):
    cur = cur_pose()
    err0 = max(abs(a - b) for a, b in zip(cur, tgt))
    print("== %s == cur=[%s] err0=%.3f" % (name, " ".join("%.3f" % x for x in cur), err0), flush=True)
    if err0 < 0.012:
        print("   已在目标(跳过)"); return True, cur
    NP = max(20, int(dur / 0.08))
    goal = FollowJointTrajectory.Goal(); traj = JointTrajectory(); traj.joint_names = NAMES
    for k in range(1, NP + 1):
        d = float(k) / NP; s = d * d * (3 - 2 * d); ds = 6 * d * (1 - d)
        p = JointTrajectoryPoint()
        p.positions = [cur[i] + (tgt[i] - cur[i]) * s for i in range(6)]
        p.velocities = [(tgt[i] - cur[i]) / dur * ds for i in range(6)]
        tf = dur * d; sec = int(tf)
        p.time_from_start = Duration(sec=sec, nanosec=int((tf - sec) * 1e9))
        traj.points.append(p)
    goal.trajectory = traj
    send = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send, timeout_sec=8)
    gh = send.result()
    if not (gh and gh.accepted):
        print("!! %s REJECTED" % name, flush=True); return False, cur
    res = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=int(dur) + 30)
    t0 = time.time(); end = cur_pose(0.1); err = max(abs(a - b) for a, b in zip(end, tgt))
    while err > 0.02 and time.time() - t0 < wait_max:
        time.sleep(1.2); end = cur_pose(0.1); err = max(abs(a - b) for a, b in zip(end, tgt))
        print("   settling err=%.3f" % err, flush=True)
    print("== %s done. err=%.4f end=[%s]" % (name, err, " ".join("%.3f" % x for x in end)), flush=True)
    return (err <= 0.05), end

# ① 归位 -> 开箱
ok, end = send_seg("①归位->开箱", P_OPEN, 5.0, 12)
if not ok: print("!! ①失败, 中止", flush=True); sys.exit(3)
time.sleep(1.0)
# ② 开箱 -> 维护
ok, end = send_seg("②开箱->维护", P_MAINT, 3.5, 12)
if not ok: print("!! ②失败, 中止", flush=True); sys.exit(3)
time.sleep(1.0)
# 夹子门限检查（进归位前）
spin(0.5); f = fb["f"] if fb["f"] is not None else -1
print("夹子读数 finger=%.3f (需在0.13~0.26)" % f, flush=True)
if not (0.13 <= f <= 0.26):
    print("!! 夹子不在归位状态(0.188附近), 跳过归位段", flush=True); sys.exit(9)
# ③ 维护 -> 归位 (分3小段)
cur = cur_pose()
W1 = lerp(cur, P_HOME, 0.45); W2 = lerp(cur, P_HOME, 0.80)
ok, end = send_seg("③A 维护->归位(45%)", W1, 3.5, 10)
if not ok: print("!! ③A失败, 中止", flush=True); sys.exit(3)
time.sleep(0.8)
ok, end = send_seg("③B 归位(80%)", W2, 3.0, 10)
if not ok: print("!! ③B失败, 中止", flush=True); sys.exit(3)
time.sleep(0.8)
ok, end = send_seg("③C 归位(终)", P_HOME, 2.5, 25)
if not ok:
    print("!! ③C未到位, 尝试摆动救援", flush=True)
    cur = cur_pose(); w = list(cur); w[4] += 0.06
    ok2, _ = send_seg("救援:抬高J5", w, 2.0, 8)
    time.sleep(0.5)
    ok3, end = send_seg("救援:再归位", P_HOME, 3.0, 25)
    if not ok3: print("!! 救援失败", flush=True); sys.exit(4)
fin = max(abs(a - b) for a, b in zip(end, P_HOME))
print("ALL DONE final_err=%.4f" % fin, flush=True)
PYEOF
chmod +x /home/Agi/jogd/seqmove4.py
pkill -f 'jogd/samp2[.]py' 2>/dev/null; sleep 0.5
nohup bash /home/Agi/jogd/samp2_run.sh 100 /home/Agi/cap_run2.csv >/tmp/cap_run2.log 2>&1 &
sleep 2
echo "capture rows: $(wc -l < /home/Agi/cap_run2.csv 2>/dev/null || echo NONE)"
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///factory/default.xml
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/Agi/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH=/home/Agi/install/lib:$LD_LIBRARY_PATH
rm -f /home/Agi/seq_run4.out
setsid nohup bash -c 'python3 -u /home/Agi/jogd/seqmove4.py > /home/Agi/seq_run4.out 2>&1; echo EXIT=$? >> /home/Agi/seq_run4.out' </dev/null &
echo "LAUNCHED (detached)"
