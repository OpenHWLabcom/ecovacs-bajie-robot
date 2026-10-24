#!/usr/bin/env python3
# seqmove.py — 顺序多段动作: 依次到 4 个姿态; 每段单发多点轨迹(smoothstep), 等完成并校验
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
    ("开箱姿态->臂归位",   [0.0, 2.46, 3.256, -0.007, -2.543, 0.0], 3.0),
    ("臂归位->开箱姿态",   [0.0, 1.766, 2.887, 0.0, -0.451, 0.0],   3.0),
    ("开箱姿态->夹爪维护", [0.045, 2.452, 2.041, -0.023, 0.232, 0.0], 2.0),
    ("夹爪维护->臂归位",   [0.0, 2.46, 3.256, -0.007, -2.543, 0.0], 4.0),
]
rclpy.init(); node = Node("seqmove")
fb = {"p": None}
def on_js(m):
    d = dict(zip(m.name, m.position)); v = [d.get(n) for n in NAMES]
    if all(x is not None for x in v): fb["p"] = [float(x) for x in v]
node.create_subscription(JointState, "/joint_states", on_js, 50)

def cur_pose():
    t0 = time.time()
    while fb["p"] is None and time.time() - t0 < 8:
        rclpy.spin_once(node, timeout_sec=0.1)
    last = fb["p"]; t0 = time.time()
    while time.time() - t0 < 0.5:
        rclpy.spin_once(node, timeout_sec=0.1); last = fb["p"]
    return last

ac = ActionClient(node, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
if not ac.wait_for_server(timeout_sec=10):
    print("NO ACTION SERVER"); sys.exit(2)

for name, tgt, dur in SEGS:
    cur = cur_pose()
    print("== %s == cur=%s" % (name, ["%.3f" % x for x in cur]), flush=True)
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
    rclpy.spin_until_future_complete(node, res, timeout_sec=int(dur) + 25)
    time.sleep(1.2)
    end = cur_pose()
    err = max(abs(a - b) for a, b in zip(end, tgt))
    print("== %s done. err=%.4f end=%s" % (name, err, ["%.3f" % x for x in end]), flush=True)
    if err > 0.05:
        print("!! %s 未到位(err>0.05), 中止" % name); sys.exit(4)
print("ALL SEGMENTS DONE", flush=True)
