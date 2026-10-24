#!/bin/bash
cat > /home/Agi/jogd/seqmove2.py <<'PYEOF'
#!/usr/bin/env python3
# seqmove2.py — 顺序多段动作 v2: 慢速 + 收敛等待 + 中止保护
import sys, time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

NAMES = ["joint%d" % i for i in range(1, 7)]
SEGS = [
    ("维护->臂归位",   [0.0, 2.46, 3.256, -0.007, -2.543, 0.0], 6.0),
    ("臂归位->开箱",   [0.0, 1.766, 2.887, 0.0, -0.451, 0.0],   5.5),
    ("开箱->夹爪维护", [0.045, 2.452, 2.041, -0.023, 0.232, 0.0], 3.0),
    ("夹爪维护->臂归位", [0.0, 2.46, 3.256, -0.007, -2.543, 0.0], 6.0),
]
rclpy.init(); node = Node("seqmove2")
fb = {"p": None}
def on_js(m):
    d = dict(zip(m.name, m.position)); v = [d.get(n) for n in NAMES]
    if all(x is not None for x in v): fb["p"] = [float(x) for x in v]
node.create_subscription(JointState, "/joint_states", on_js, 50)

def cur_pose(sec=0.5):
    t0 = time.time()
    while fb["p"] is None and time.time() - t0 < 8:
        rclpy.spin_once(node, timeout_sec=0.1)
    last = fb["p"]; t0 = time.time()
    while time.time() - t0 < sec:
        rclpy.spin_once(node, timeout_sec=0.1); last = fb["p"]
    return last

ac = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
if not ac.wait_for_server(timeout_sec=10):
    print("NO ACTION SERVER"); sys.exit(2)

for name, tgt, dur in SEGS:
    cur = cur_pose()
    err0 = max(abs(a - b) for a, b in zip(cur, tgt))
    print("== %s == cur=%s err0=%.3f" % (name, ["%.3f" % x for x in cur], err0), flush=True)
    if err0 < 0.01:
        print("   已在目标位, 跳过"); continue
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
        print("!! %s REJECTED" % name); sys.exit(3)
    res = gh.get_result_async()
    rclpy.spin_until_future_complete(node, res, timeout_sec=int(dur) + 30)
    t0 = time.time(); end = cur_pose(0.1); err = max(abs(a - b) for a, b in zip(end, tgt))
    while err > 0.02 and time.time() - t0 < 12:
        time.sleep(1.5); end = cur_pose(0.1); err = max(abs(a - b) for a, b in zip(end, tgt))
        print("   settling... err=%.3f" % err, flush=True)
    print("== %s done. err=%.4f end=%s" % (name, err, ["%.3f" % x for x in end]), flush=True)
    if err > 0.05:
        print("!! %s 未到位(err>0.05), 中止" % name); sys.exit(4)
    time.sleep(1.0)
print("ALL SEGMENTS DONE", flush=True)
PYEOF
chmod +x /home/Agi/jogd/seqmove2.py
pkill -f 'jogd/samp2[.]py' 2>/dev/null; pkill -f 'jogd/samp[.]py' 2>/dev/null; sleep 0.5
nohup bash /home/Agi/jogd/samp2_run.sh 80 /home/Agi/cap_scheme.csv >/tmp/cap_scheme.log 2>&1 &
sleep 3
echo "capture rows: $(wc -l < /home/Agi/cap_scheme.csv 2>/dev/null || echo NONE)"
export PATH=/usr/bin:/bin:/usr/sbin:/sbin
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///factory/default.xml
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/Agi/install/setup.bash 2>/dev/null
export LD_LIBRARY_PATH=/home/Agi/install/lib:$LD_LIBRARY_PATH
python3 -u /home/Agi/jogd/seqmove2.py
echo "SEQ_EXIT=$?"
